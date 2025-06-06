// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2015-2023, Linaro Limited
 * Copyright (c) 2023, Arm Limited
 */

#include <arm.h>
#include <assert.h>
#include <compiler.h>
#include <config.h>
#include <console.h>
#include <crypto/crypto.h>
#include <drivers/gic.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>
#include <ffa.h>
#include <initcall.h>
#include <inttypes.h>
#include <io.h>
#include <keep.h>
#include <kernel/asan.h>
#include <kernel/boot.h>
#include <kernel/dt.h>
#include <kernel/linker.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/tee_misc.h>
#include <kernel/thread.h>
#include <kernel/tpm.h>
#include <kernel/transfer_list.h>
#include <libfdt.h>
#include <malloc.h>
#include <memtag.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <mm/fobj.h>
#include <mm/page_alloc.h>
#include <mm/phys_mem.h>
#include <mm/tee_mm.h>
#include <mm/tee_pager.h>
#include <sm/psci.h>
#include <stdalign.h>
#include <trace.h>
#include <utee_defines.h>
#include <util.h>

#include <platform_config.h>

#if !defined(CFG_WITH_ARM_TRUSTED_FW)
#include <sm/sm.h>
#endif

#if defined(CFG_WITH_VFP)
#include <kernel/vfp.h>
#endif

/*
 * In this file we're using unsigned long to represent physical pointers as
 * they are received in a single register when OP-TEE is initially entered.
 * This limits 32-bit systems to only use make use of the lower 32 bits
 * of a physical address for initial parameters.
 *
 * 64-bit systems on the other hand can use full 64-bit physical pointers.
 */
#define PADDR_INVALID		ULONG_MAX

#if defined(CFG_BOOT_SECONDARY_REQUEST)
struct ns_entry_context {
	uintptr_t entry_point;
	uintptr_t context_id;
};
struct ns_entry_context ns_entry_contexts[CFG_TEE_CORE_NB_CORE];
static uint32_t spin_table[CFG_TEE_CORE_NB_CORE];
#endif

#ifdef CFG_BOOT_SYNC_CPU
/*
 * Array used when booting, to synchronize cpu.
 * When 0, the cpu has not started.
 * When 1, it has started
 */
uint32_t sem_cpu_sync[CFG_TEE_CORE_NB_CORE];
DECLARE_KEEP_PAGER(sem_cpu_sync);
#endif

/*
 * Must not be in .bss since it's initialized and used from assembly before
 * .bss is cleared.
 */
vaddr_t boot_cached_mem_end __nex_data = 1;

static unsigned long boot_arg_fdt __nex_bss;
unsigned long boot_arg_nsec_entry __nex_bss;
static unsigned long boot_arg_pageable_part __nex_bss;
static unsigned long boot_arg_transfer_list __nex_bss;
static struct transfer_list_header *mapped_tl __nex_bss;

#ifdef CFG_SECONDARY_INIT_CNTFRQ
static uint32_t cntfrq;
#endif

/* May be overridden in plat-$(PLATFORM)/main.c */
__weak void plat_primary_init_early(void)
{
}
DECLARE_KEEP_PAGER(plat_primary_init_early);

/* May be overridden in plat-$(PLATFORM)/main.c */
__weak void boot_primary_init_intc(void)
{
}

/* May be overridden in plat-$(PLATFORM)/main.c */
__weak void boot_secondary_init_intc(void)
{
}

/* May be overridden in plat-$(PLATFORM)/main.c */
__weak unsigned long plat_get_aslr_seed(void)
{
	DMSG("Warning: no ASLR seed");

	return 0;
}

/*
 * This function is called as a guard after each smc call which is not
 * supposed to return.
 */
void __panic_at_smc_return(void)
{
	panic();
}

#if defined(CFG_WITH_ARM_TRUSTED_FW)
void init_sec_mon(unsigned long nsec_entry __maybe_unused)
{
	assert(nsec_entry == PADDR_INVALID);
	/* Do nothing as we don't have a secure monitor */
}
#else
/* May be overridden in plat-$(PLATFORM)/main.c */
__weak void init_sec_mon(unsigned long nsec_entry)
{
	struct sm_nsec_ctx *nsec_ctx;

	assert(nsec_entry != PADDR_INVALID);

	/* Initialize secure monitor */
	nsec_ctx = sm_get_nsec_ctx();
	nsec_ctx->mon_lr = nsec_entry;
	nsec_ctx->mon_spsr = CPSR_MODE_SVC | CPSR_I;
	if (nsec_entry & 1)
		nsec_ctx->mon_spsr |= CPSR_T;
}
#endif

#if defined(CFG_WITH_ARM_TRUSTED_FW)
static void init_vfp_nsec(void)
{
}
#else
static void init_vfp_nsec(void)
{
	/* Normal world can use CP10 and CP11 (SIMD/VFP) */
	write_nsacr(read_nsacr() | NSACR_CP10 | NSACR_CP11);
}
#endif

static void check_crypto_extensions(void)
{
	bool ce_supported = true;

	if (!feat_aes_implemented() &&
	    IS_ENABLED(CFG_CRYPTO_AES_ARM_CE)) {
		EMSG("AES instructions are not supported");
		ce_supported = false;
	}

	if (!feat_sha1_implemented() &&
	    IS_ENABLED(CFG_CRYPTO_SHA1_ARM_CE)) {
		EMSG("SHA1 instructions are not supported");
		ce_supported = false;
	}

	if (!feat_sha256_implemented() &&
	    IS_ENABLED(CFG_CRYPTO_SHA256_ARM_CE)) {
		EMSG("SHA256 instructions are not supported");
		ce_supported = false;
	}

	/* Check aarch64 specific instructions */
	if (IS_ENABLED(CFG_ARM64_core)) {
		if (!feat_sha512_implemented() &&
		    IS_ENABLED(CFG_CRYPTO_SHA512_ARM_CE)) {
			EMSG("SHA512 instructions are not supported");
			ce_supported = false;
		}

		if (!feat_sha3_implemented() &&
		    IS_ENABLED(CFG_CRYPTO_SHA3_ARM_CE)) {
			EMSG("SHA3 instructions are not supported");
			ce_supported = false;
		}

		if (!feat_sm3_implemented() &&
		    IS_ENABLED(CFG_CRYPTO_SM3_ARM_CE)) {
			EMSG("SM3 instructions are not supported");
			ce_supported = false;
		}

		if (!feat_sm4_implemented() &&
		    IS_ENABLED(CFG_CRYPTO_SM4_ARM_CE)) {
			EMSG("SM4 instructions are not supported");
			ce_supported = false;
		}
	}

	if (!ce_supported)
		panic("HW doesn't support CE instructions");
}

#if defined(CFG_WITH_VFP)

#ifdef ARM32
static void init_vfp_sec(void)
{
	uint32_t cpacr = read_cpacr();

	/*
	 * Enable Advanced SIMD functionality.
	 * Enable use of D16-D31 of the Floating-point Extension register
	 * file.
	 */
	cpacr &= ~(CPACR_ASEDIS | CPACR_D32DIS);
	/*
	 * Enable usage of CP10 and CP11 (SIMD/VFP) (both kernel and user
	 * mode.
	 */
	cpacr |= CPACR_CP(10, CPACR_CP_ACCESS_FULL);
	cpacr |= CPACR_CP(11, CPACR_CP_ACCESS_FULL);
	write_cpacr(cpacr);
}
#endif /* ARM32 */

#ifdef ARM64
static void init_vfp_sec(void)
{
	/* Not using VFP until thread_kernel_enable_vfp() */
	vfp_disable();
}
#endif /* ARM64 */

#else /* CFG_WITH_VFP */

static void init_vfp_sec(void)
{
	/* Not using VFP */
}
#endif

#ifdef CFG_SECONDARY_INIT_CNTFRQ
static void primary_save_cntfrq(void)
{
	assert(cntfrq == 0);

	/*
	 * CNTFRQ should be initialized on the primary CPU by a
	 * previous boot stage
	 */
	cntfrq = read_cntfrq();
}

static void secondary_init_cntfrq(void)
{
	assert(cntfrq != 0);
	write_cntfrq(cntfrq);
}
#else /* CFG_SECONDARY_INIT_CNTFRQ */
static void primary_save_cntfrq(void)
{
}

static void secondary_init_cntfrq(void)
{
}
#endif

#ifdef CFG_CORE_SANITIZE_KADDRESS
static void init_run_constructors(void)
{
	const vaddr_t *ctor;

	for (ctor = &__ctor_list; ctor < &__ctor_end; ctor++)
		((void (*)(void))(*ctor))();
}

static void init_asan(void)
{

	/*
	 * CFG_ASAN_SHADOW_OFFSET is also supplied as
	 * -fasan-shadow-offset=$(CFG_ASAN_SHADOW_OFFSET) to the compiler.
	 * Since all the needed values to calculate the value of
	 * CFG_ASAN_SHADOW_OFFSET isn't available in to make we need to
	 * calculate it in advance and hard code it into the platform
	 * conf.mk. Here where we have all the needed values we double
	 * check that the compiler is supplied the correct value.
	 */

#define __ASAN_SHADOW_START \
	ROUNDUP(TEE_RAM_START + (TEE_RAM_VA_SIZE * 8) / 9 - 8, 8)
	assert(__ASAN_SHADOW_START == (vaddr_t)&__asan_shadow_start);
#define __CFG_ASAN_SHADOW_OFFSET \
	(__ASAN_SHADOW_START - (TEE_RAM_START / 8))
	COMPILE_TIME_ASSERT(CFG_ASAN_SHADOW_OFFSET == __CFG_ASAN_SHADOW_OFFSET);
#undef __ASAN_SHADOW_START
#undef __CFG_ASAN_SHADOW_OFFSET

	/*
	 * Assign area covered by the shadow area, everything from start up
	 * to the beginning of the shadow area.
	 */
	asan_set_shadowed((void *)TEE_LOAD_ADDR, &__asan_shadow_start);

	/*
	 * Add access to areas that aren't opened automatically by a
	 * constructor.
	 */
	boot_mem_init_asan();
	asan_tag_access(&__ctor_list, &__ctor_end);
	asan_tag_access(__rodata_start, __rodata_end);
#ifdef CFG_WITH_PAGER
	asan_tag_access(__pageable_start, __pageable_end);
#endif /*CFG_WITH_PAGER*/
	asan_tag_access(__nozi_start, __nozi_end);
#ifdef ARM32
	asan_tag_access(__exidx_start, __exidx_end);
	asan_tag_access(__extab_start, __extab_end);
#endif

	init_run_constructors();

	/* Everything is tagged correctly, let's start address sanitizing. */
	asan_start();
}
#else /*CFG_CORE_SANITIZE_KADDRESS*/
static void init_asan(void)
{
}
#endif /*CFG_CORE_SANITIZE_KADDRESS*/

#if defined(CFG_MEMTAG)
/* Called from entry_a64.S only when MEMTAG is configured */
void boot_init_memtag(void)
{
	memtag_init_ops(feat_mte_implemented());
}

static TEE_Result mmap_clear_memtag(struct tee_mmap_region *map,
				    void *ptr __unused)
{
	switch (map->type) {
	case MEM_AREA_NEX_RAM_RO:
	case MEM_AREA_SEC_RAM_OVERALL:
		DMSG("Clearing tags for VA %#"PRIxVA"..%#"PRIxVA,
		     map->va, map->va + map->size - 1);
		memtag_set_tags((void *)map->va, map->size, 0);
		break;
	default:
		break;
	}

	return TEE_SUCCESS;
}

/* Called from entry_a64.S only when MEMTAG is configured */
void boot_clear_memtag(void)
{
	core_mmu_for_each_map(NULL, mmap_clear_memtag);
}
#endif

#ifdef CFG_WITH_PAGER

#ifdef CFG_CORE_SANITIZE_KADDRESS
static void carve_out_asan_mem(void)
{
	nex_phys_mem_partial_carve_out(ASAN_MAP_PA, ASAN_MAP_SZ);
}
#else
static void carve_out_asan_mem(void)
{
}
#endif

static void print_pager_pool_size(void)
{
	struct tee_pager_stats __maybe_unused stats;

	tee_pager_get_stats(&stats);
	IMSG("Pager pool size: %zukB",
		stats.npages_all * SMALL_PAGE_SIZE / 1024);
}

static void init_virt_pool(tee_mm_pool_t *virt_pool)
{
	const vaddr_t begin = VCORE_START_VA;
	size_t size = TEE_RAM_VA_SIZE;

#ifdef CFG_CORE_SANITIZE_KADDRESS
	/* Carve out asan memory, flat maped after core memory */
	if (begin + size > ASAN_SHADOW_PA)
		size = ASAN_MAP_PA - begin;
#endif

	if (!tee_mm_init(virt_pool, begin, size, SMALL_PAGE_SHIFT,
			 TEE_MM_POOL_NO_FLAGS))
		panic("core_virt_mem_pool init failed");
}

/*
 * With CFG_CORE_ASLR=y the init part is relocated very early during boot.
 * The init part is also paged just as the rest of the normal paged code, with
 * the difference that it's preloaded during boot. When the backing store
 * is configured the entire paged binary is copied in place and then also
 * the init part. Since the init part has been relocated (references to
 * addresses updated to compensate for the new load address) this has to be
 * undone for the hashes of those pages to match with the original binary.
 *
 * If CFG_CORE_ASLR=n, nothing needs to be done as the code/ro pages are
 * unchanged.
 */
static void undo_init_relocation(uint8_t *paged_store __maybe_unused)
{
#ifdef CFG_CORE_ASLR
	unsigned long *ptr = NULL;
	const uint32_t *reloc = NULL;
	const uint32_t *reloc_end = NULL;
	unsigned long offs = boot_mmu_config.map_offset;
	const struct boot_embdata *embdata = (const void *)__init_end;
	vaddr_t addr_end = (vaddr_t)__init_end - offs - TEE_LOAD_ADDR;
	vaddr_t addr_start = (vaddr_t)__init_start - offs - TEE_LOAD_ADDR;

	reloc = (const void *)((vaddr_t)embdata + embdata->reloc_offset);
	reloc_end = reloc + embdata->reloc_len / sizeof(*reloc);

	for (; reloc < reloc_end; reloc++) {
		if (*reloc < addr_start)
			continue;
		if (*reloc >= addr_end)
			break;
		ptr = (void *)(paged_store + *reloc - addr_start);
		*ptr -= offs;
	}
#endif
}

static struct fobj *ro_paged_alloc(tee_mm_entry_t *mm, void *hashes,
				   void *store)
{
	const unsigned int num_pages = tee_mm_get_bytes(mm) / SMALL_PAGE_SIZE;
#ifdef CFG_CORE_ASLR
	unsigned int reloc_offs = (vaddr_t)__pageable_start - VCORE_START_VA;
	const struct boot_embdata *embdata = (const void *)__init_end;
	const void *reloc = __init_end + embdata->reloc_offset;

	return fobj_ro_reloc_paged_alloc(num_pages, hashes, reloc_offs,
					 reloc, embdata->reloc_len, store);
#else
	return fobj_ro_paged_alloc(num_pages, hashes, store);
#endif
}

static void init_pager_runtime(unsigned long pageable_part)
{
	size_t n;
	size_t init_size = (size_t)(__init_end - __init_start);
	size_t pageable_start = (size_t)__pageable_start;
	size_t pageable_end = (size_t)__pageable_end;
	size_t pageable_size = pageable_end - pageable_start;
	vaddr_t tzsram_end = TZSRAM_BASE + TZSRAM_SIZE - TEE_LOAD_ADDR +
			     VCORE_START_VA;
	size_t hash_size = (pageable_size / SMALL_PAGE_SIZE) *
			   TEE_SHA256_HASH_SIZE;
	const struct boot_embdata *embdata = (const void *)__init_end;
	const void *tmp_hashes = NULL;
	tee_mm_entry_t *mm = NULL;
	struct fobj *fobj = NULL;
	uint8_t *paged_store = NULL;
	uint8_t *hashes = NULL;

	assert(pageable_size % SMALL_PAGE_SIZE == 0);
	assert(embdata->total_len >= embdata->hashes_offset +
				     embdata->hashes_len);
	assert(hash_size == embdata->hashes_len);

	tmp_hashes = __init_end + embdata->hashes_offset;

	/*
	 * This needs to be initialized early to support address lookup
	 * in MEM_AREA_TEE_RAM
	 */
	tee_pager_early_init();

	hashes = malloc(hash_size);
	IMSG_RAW("\n");
	IMSG("Pager is enabled. Hashes: %zu bytes", hash_size);
	assert(hashes);
	asan_memcpy_unchecked(hashes, tmp_hashes, hash_size);

	/*
	 * The pager is about the be enabled below, eventual temporary boot
	 * memory allocation must be removed now.
	 */
	boot_mem_release_tmp_alloc();

	carve_out_asan_mem();

	mm = nex_phys_mem_ta_alloc(pageable_size);
	assert(mm);
	paged_store = phys_to_virt(tee_mm_get_smem(mm),
				   MEM_AREA_SEC_RAM_OVERALL, pageable_size);
	/*
	 * Load pageable part in the dedicated allocated area:
	 * - Move pageable non-init part into pageable area. Note bootloader
	 *   may have loaded it anywhere in TA RAM hence use memmove().
	 * - Copy pageable init part from current location into pageable area.
	 */
	memmove(paged_store + init_size,
		phys_to_virt(pageable_part,
			     core_mmu_get_type_by_pa(pageable_part),
			     __pageable_part_end - __pageable_part_start),
		__pageable_part_end - __pageable_part_start);
	asan_memcpy_unchecked(paged_store, __init_start, init_size);
	/*
	 * Undo eventual relocation for the init part so the hash checks
	 * can pass.
	 */
	undo_init_relocation(paged_store);

	/* Check that hashes of what's in pageable area is OK */
	DMSG("Checking hashes of pageable area");
	for (n = 0; (n * SMALL_PAGE_SIZE) < pageable_size; n++) {
		const uint8_t *hash = hashes + n * TEE_SHA256_HASH_SIZE;
		const uint8_t *page = paged_store + n * SMALL_PAGE_SIZE;
		TEE_Result res;

		DMSG("hash pg_idx %zu hash %p page %p", n, hash, page);
		res = hash_sha256_check(hash, page, SMALL_PAGE_SIZE);
		if (res != TEE_SUCCESS) {
			EMSG("Hash failed for page %zu at %p: res 0x%x",
			     n, (void *)page, res);
			panic();
		}
	}

	/*
	 * Assert prepaged init sections are page aligned so that nothing
	 * trails uninited at the end of the premapped init area.
	 */
	assert(!(init_size & SMALL_PAGE_MASK));

	/*
	 * Initialize the virtual memory pool used for main_mmu_l2_ttb which
	 * is supplied to tee_pager_init() below.
	 */
	init_virt_pool(&core_virt_mem_pool);

	/*
	 * Assign alias area for pager end of the small page block the rest
	 * of the binary is loaded into. We're taking more than needed, but
	 * we're guaranteed to not need more than the physical amount of
	 * TZSRAM.
	 */
	mm = tee_mm_alloc2(&core_virt_mem_pool,
			   (vaddr_t)core_virt_mem_pool.lo +
			   core_virt_mem_pool.size - TZSRAM_SIZE,
			   TZSRAM_SIZE);
	assert(mm);
	tee_pager_set_alias_area(mm);

	/*
	 * Claim virtual memory which isn't paged.
	 * Linear memory (flat map core memory) ends there.
	 */
	mm = tee_mm_alloc2(&core_virt_mem_pool, VCORE_UNPG_RX_PA,
			   (vaddr_t)(__pageable_start - VCORE_UNPG_RX_PA));
	assert(mm);

	/*
	 * Allocate virtual memory for the pageable area and let the pager
	 * take charge of all the pages already assigned to that memory.
	 */
	mm = tee_mm_alloc2(&core_virt_mem_pool, (vaddr_t)__pageable_start,
			   pageable_size);
	assert(mm);
	fobj = ro_paged_alloc(mm, hashes, paged_store);
	assert(fobj);
	tee_pager_add_core_region(tee_mm_get_smem(mm), PAGED_REGION_TYPE_RO,
				  fobj);
	fobj_put(fobj);

	tee_pager_add_pages(pageable_start, init_size / SMALL_PAGE_SIZE, false);
	tee_pager_add_pages(pageable_start + init_size,
			    (pageable_size - init_size) / SMALL_PAGE_SIZE,
			    true);
	if (pageable_end < tzsram_end)
		tee_pager_add_pages(pageable_end, (tzsram_end - pageable_end) /
						   SMALL_PAGE_SIZE, true);

	/*
	 * There may be physical pages in TZSRAM before the core load address.
	 * These pages can be added to the physical pages pool of the pager.
	 * This setup may happen when a the secure bootloader runs in TZRAM
	 * and its memory can be reused by OP-TEE once boot stages complete.
	 */
	tee_pager_add_pages(core_virt_mem_pool.lo,
			    (VCORE_UNPG_RX_PA - core_virt_mem_pool.lo) /
				SMALL_PAGE_SIZE,
			    true);

	print_pager_pool_size();
}
#else /*!CFG_WITH_PAGER*/
static void init_pager_runtime(unsigned long pageable_part __unused)
{
}
#endif

#if defined(CFG_DT)
static int add_optee_dt_node(struct dt_descriptor *dt)
{
	int offs;
	int ret;

	if (fdt_path_offset(dt->blob, "/firmware/optee") >= 0) {
		DMSG("OP-TEE Device Tree node already exists!");
		return 0;
	}

	offs = fdt_path_offset(dt->blob, "/firmware");
	if (offs < 0) {
		offs = add_dt_path_subnode(dt, "/", "firmware");
		if (offs < 0)
			return -1;
	}

	offs = fdt_add_subnode(dt->blob, offs, "optee");
	if (offs < 0)
		return -1;

	ret = fdt_setprop_string(dt->blob, offs, "compatible",
				 "linaro,optee-tz");
	if (ret < 0)
		return -1;
	ret = fdt_setprop_string(dt->blob, offs, "method", "smc");
	if (ret < 0)
		return -1;

	if (CFG_CORE_ASYNC_NOTIF_GIC_INTID) {
		/*
		 * The format of the interrupt property is defined by the
		 * binding of the interrupt domain root. In this case it's
		 * one Arm GIC v1, v2 or v3 so we must be compatible with
		 * these.
		 *
		 * An SPI type of interrupt is indicated with a 0 in the
		 * first cell. A PPI type is indicated with value 1.
		 *
		 * The interrupt number goes in the second cell where
		 * SPIs ranges from 0 to 987 and PPI ranges from 0 to 15.
		 *
		 * Flags are passed in the third cells.
		 */
		uint32_t itr_trigger = 0;
		uint32_t itr_type = 0;
		uint32_t itr_id = 0;
		uint32_t val[3] = { };

		/* PPI are visible only in current CPU cluster */
		static_assert(IS_ENABLED(CFG_CORE_FFA) ||
			      !CFG_CORE_ASYNC_NOTIF_GIC_INTID ||
			      (CFG_CORE_ASYNC_NOTIF_GIC_INTID >=
			       GIC_SPI_BASE) ||
			      ((CFG_TEE_CORE_NB_CORE <= 8) &&
			       (CFG_CORE_ASYNC_NOTIF_GIC_INTID >=
				GIC_PPI_BASE)));

		if (CFG_CORE_ASYNC_NOTIF_GIC_INTID >= GIC_SPI_BASE) {
			itr_type = GIC_SPI;
			itr_id = CFG_CORE_ASYNC_NOTIF_GIC_INTID - GIC_SPI_BASE;
			itr_trigger = IRQ_TYPE_EDGE_RISING;
		} else {
			itr_type = GIC_PPI;
			itr_id = CFG_CORE_ASYNC_NOTIF_GIC_INTID - GIC_PPI_BASE;
			itr_trigger = IRQ_TYPE_EDGE_RISING |
				      GIC_CPU_MASK_SIMPLE(CFG_TEE_CORE_NB_CORE);
		}

		val[0] = TEE_U32_TO_BIG_ENDIAN(itr_type);
		val[1] = TEE_U32_TO_BIG_ENDIAN(itr_id);
		val[2] = TEE_U32_TO_BIG_ENDIAN(itr_trigger);

		ret = fdt_setprop(dt->blob, offs, "interrupts", val,
				  sizeof(val));
		if (ret < 0)
			return -1;
	}
	return 0;
}

#ifdef CFG_PSCI_ARM32
static int append_psci_compatible(void *fdt, int offs, const char *str)
{
	return fdt_appendprop(fdt, offs, "compatible", str, strlen(str) + 1);
}

static int dt_add_psci_node(struct dt_descriptor *dt)
{
	int offs;

	if (fdt_path_offset(dt->blob, "/psci") >= 0) {
		DMSG("PSCI Device Tree node already exists!");
		return 0;
	}

	offs = add_dt_path_subnode(dt, "/", "psci");
	if (offs < 0)
		return -1;
	if (append_psci_compatible(dt->blob, offs, "arm,psci-1.0"))
		return -1;
	if (append_psci_compatible(dt->blob, offs, "arm,psci-0.2"))
		return -1;
	if (append_psci_compatible(dt->blob, offs, "arm,psci"))
		return -1;
	if (fdt_setprop_string(dt->blob, offs, "method", "smc"))
		return -1;
	if (fdt_setprop_u32(dt->blob, offs, "cpu_suspend", PSCI_CPU_SUSPEND))
		return -1;
	if (fdt_setprop_u32(dt->blob, offs, "cpu_off", PSCI_CPU_OFF))
		return -1;
	if (fdt_setprop_u32(dt->blob, offs, "cpu_on", PSCI_CPU_ON))
		return -1;
	if (fdt_setprop_u32(dt->blob, offs, "sys_poweroff", PSCI_SYSTEM_OFF))
		return -1;
	if (fdt_setprop_u32(dt->blob, offs, "sys_reset", PSCI_SYSTEM_RESET))
		return -1;
	return 0;
}

static int check_node_compat_prefix(struct dt_descriptor *dt, int offs,
				    const char *prefix)
{
	const size_t prefix_len = strlen(prefix);
	size_t l;
	int plen;
	const char *prop;

	prop = fdt_getprop(dt->blob, offs, "compatible", &plen);
	if (!prop)
		return -1;

	while (plen > 0) {
		if (memcmp(prop, prefix, prefix_len) == 0)
			return 0; /* match */

		l = strlen(prop) + 1;
		prop += l;
		plen -= l;
	}

	return -1;
}

static int dt_add_psci_cpu_enable_methods(struct dt_descriptor *dt)
{
	int offs = 0;

	while (1) {
		offs = fdt_next_node(dt->blob, offs, NULL);
		if (offs < 0)
			break;
		if (fdt_getprop(dt->blob, offs, "enable-method", NULL))
			continue; /* already set */
		if (check_node_compat_prefix(dt, offs, "arm,cortex-a"))
			continue; /* no compatible */
		if (fdt_setprop_string(dt->blob, offs, "enable-method", "psci"))
			return -1;
		/* Need to restart scanning as offsets may have changed */
		offs = 0;
	}
	return 0;
}

static int config_psci(struct dt_descriptor *dt)
{
	if (dt_add_psci_node(dt))
		return -1;
	return dt_add_psci_cpu_enable_methods(dt);
}
#else
static int config_psci(struct dt_descriptor *dt __unused)
{
	return 0;
}
#endif /*CFG_PSCI_ARM32*/

static int mark_tzdram_as_reserved(struct dt_descriptor *dt)
{
	return add_res_mem_dt_node(dt, "optee_core", CFG_TZDRAM_START,
				   CFG_TZDRAM_SIZE);
}

static void update_external_dt(void)
{
	struct dt_descriptor *dt = get_external_dt_desc();

	if (!dt || !dt->blob)
		return;

	if (!IS_ENABLED(CFG_CORE_FFA) && add_optee_dt_node(dt))
		panic("Failed to add OP-TEE Device Tree node");

	if (config_psci(dt))
		panic("Failed to config PSCI");

#ifdef CFG_CORE_RESERVED_SHM
	if (mark_static_shm_as_reserved(dt))
		panic("Failed to config non-secure memory");
#endif

	if (mark_tzdram_as_reserved(dt))
		panic("Failed to config secure memory");
}
#else /*CFG_DT*/
static void update_external_dt(void)
{
}
#endif /*!CFG_DT*/

void init_tee_runtime(void)
{
	/*
	 * With virtualization we call this function when creating the
	 * OP-TEE partition instead.
	 */
	if (!IS_ENABLED(CFG_NS_VIRTUALIZATION))
		call_preinitcalls();
	call_early_initcalls();
	call_service_initcalls();

	/*
	 * These two functions uses crypto_rng_read() to initialize the
	 * pauth keys. Once call_initcalls() returns we're guaranteed that
	 * crypto_rng_read() is ready to be used.
	 */
	thread_init_core_local_pauth_keys();
	thread_init_thread_pauth_keys();

	/*
	 * Reinitialize canaries around the stacks with crypto_rng_read().
	 *
	 * TODO: Updating canaries when CFG_NS_VIRTUALIZATION is enabled will
	 * require synchronization between thread_check_canaries() and
	 * thread_update_canaries().
	 */
	if (!IS_ENABLED(CFG_NS_VIRTUALIZATION))
		thread_update_canaries();
}

static bool add_padding_to_pool(vaddr_t va, size_t len, void *ptr __unused)
{
#ifdef CFG_NS_VIRTUALIZATION
	nex_malloc_add_pool((void *)va, len);
#else
	malloc_add_pool((void *)va, len);
#endif
	return true;
}

static void init_primary(unsigned long pageable_part)
{
	vaddr_t va = 0;

	/*
	 * Mask asynchronous exceptions before switch to the thread vector
	 * as the thread handler requires those to be masked while
	 * executing with the temporary stack. The thread subsystem also
	 * asserts that the foreign interrupts are blocked when using most of
	 * its functions.
	 */
	thread_set_exceptions(THREAD_EXCP_ALL);
	primary_save_cntfrq();
	init_vfp_sec();

	if (IS_ENABLED(CFG_CRYPTO_WITH_CE))
		check_crypto_extensions();

	init_asan();

	/*
	 * By default whole OP-TEE uses malloc, so we need to initialize
	 * it early. But, when virtualization is enabled, malloc is used
	 * only by TEE runtime, so malloc should be initialized later, for
	 * every virtual partition separately. Core code uses nex_malloc
	 * instead.
	 */
#ifdef CFG_WITH_PAGER
	/* Add heap2 first as heap1 may be too small as initial bget pool */
	malloc_add_pool(__heap2_start, __heap2_end - __heap2_start);
#endif
#ifdef CFG_NS_VIRTUALIZATION
	nex_malloc_add_pool(__nex_heap_start, __nex_heap_end -
					      __nex_heap_start);
#else
	malloc_add_pool(__heap1_start, __heap1_end - __heap1_start);
#endif
	IMSG_RAW("\n");
	if (IS_ENABLED(CFG_DYN_CONFIG)) {
		size_t sz = sizeof(struct thread_core_local) *
			    CFG_TEE_CORE_NB_CORE;
		void *p = boot_mem_alloc(sz, alignof(void *) * 2);

#ifdef CFG_NS_VIRTUALIZATION
		nex_malloc_add_pool(p, sz);
#else
		malloc_add_pool(p, sz);
#endif
	}

	core_mmu_save_mem_map();
	core_mmu_init_phys_mem();
	boot_mem_foreach_padding(add_padding_to_pool, NULL);
	va = boot_mem_release_unused();
	if (!IS_ENABLED(CFG_WITH_PAGER)) {
		/*
		 * We must update boot_cached_mem_end to reflect the memory
		 * just unmapped by boot_mem_release_unused().
		 */
		assert(va && va <= boot_cached_mem_end);
		boot_cached_mem_end = va;
	}

	if (IS_ENABLED(CFG_DYN_CONFIG)) {
		/*
		 * This is needed to enable virt_page_alloc() now that
		 * boot_mem_alloc() can't be used any longer.
		 */
		if (IS_ENABLED(CFG_NS_VIRTUALIZATION))
			nex_page_alloc_init();
		else
			page_alloc_init();
	}

	if (IS_ENABLED(CFG_WITH_PAGER)) {
		/*
		 * Pager: init_runtime() calls thread_kernel_enable_vfp()
		 * so we must set a current thread right now to avoid a
		 * chicken-and-egg problem (thread_init_boot_thread() sets
		 * the current thread but needs things set by
		 * init_runtime()).
		 */
		thread_get_core_local()->curr_thread = 0;
		init_pager_runtime(pageable_part);
	}

	/* Initialize canaries around the stacks */
	thread_init_canaries();
	thread_init_per_cpu();
}

static bool cpu_nmfi_enabled(void)
{
#if defined(ARM32)
	return read_sctlr() & SCTLR_NMFI;
#else
	/* Note: ARM64 does not feature non-maskable FIQ support. */
	return false;
#endif
}

/*
 * Note: this function is weak just to make it possible to exclude it from
 * the unpaged area.
 */
void __weak boot_init_primary_late(unsigned long fdt __unused,
				   unsigned long manifest __unused)
{
	size_t fdt_size = CFG_DTB_MAX_SIZE;

	if (IS_ENABLED(CFG_TRANSFER_LIST) && mapped_tl) {
		struct transfer_list_entry *tl_e = NULL;

		tl_e = transfer_list_find(mapped_tl, TL_TAG_FDT);
		if (tl_e) {
			/*
			 * Expand the data size of the DTB entry to the maximum
			 * allocable mapped memory to reserve sufficient space
			 * for inserting new nodes, avoid potentially corrupting
			 * next entries.
			 */
			uint32_t dtb_max_sz = mapped_tl->max_size -
					      mapped_tl->size + tl_e->data_size;

			if (!transfer_list_set_data_size(mapped_tl, tl_e,
							 dtb_max_sz)) {
				EMSG("Failed to extend DTB size to %#"PRIx32,
				     dtb_max_sz);
				panic();
			}
			fdt_size = tl_e->data_size;
		}
	}

	init_external_dt(boot_arg_fdt, fdt_size);
	reinit_manifest_dt();
#ifdef CFG_CORE_SEL1_SPMC
	tpm_map_log_area(get_manifest_dt());
#else
	tpm_map_log_area(get_external_dt());
#endif
	discover_nsec_memory();
	update_external_dt();
	configure_console_from_dt();

	if (IS_ENABLED(CFG_NS_VIRTUALIZATION)) {
		/*
		 * Virtualization: We can't initialize threads right now because
		 * threads belong to "tee" part and will be initialized
		 * separately per each new virtual guest. So, we'll clear
		 * "curr_thread" and call it done.
		 */
		thread_get_core_local()->curr_thread = -1;
	} else {
		thread_init_threads(CFG_NUM_THREADS);
		thread_init_boot_thread();
	}
	thread_init_thread_core_local(CFG_TEE_CORE_NB_CORE);
}

void __weak boot_init_primary_runtime(void)
{
	thread_init_primary();
	IMSG("OP-TEE version: %s", core_v_str);
	if (IS_ENABLED(CFG_INSECURE)) {
		IMSG("WARNING: This OP-TEE configuration might be insecure!");
		IMSG("WARNING: Please check https://optee.readthedocs.io/en/latest/architecture/porting_guidelines.html");
	}
	IMSG("Primary CPU initializing");
#ifdef CFG_CORE_ASLR
	DMSG("Executing at offset %#lx with virtual load address %#"PRIxVA,
	     (unsigned long)boot_mmu_config.map_offset, VCORE_START_VA);
#endif
#ifdef CFG_NS_VIRTUALIZATION
	DMSG("NS-virtualization enabled, supporting %u guests",
	     CFG_VIRT_GUEST_COUNT);
#endif
	if (IS_ENABLED(CFG_MEMTAG))
		DMSG("Memory tagging %s",
		     memtag_is_enabled() ?  "enabled" : "disabled");

	/* Check if platform needs NMFI workaround */
	if (cpu_nmfi_enabled())	{
		if (!IS_ENABLED(CFG_CORE_WORKAROUND_ARM_NMFI))
			IMSG("WARNING: This ARM core has NMFI enabled, please apply workaround!");
	} else {
		if (IS_ENABLED(CFG_CORE_WORKAROUND_ARM_NMFI))
			IMSG("WARNING: This ARM core does not have NMFI enabled, no need for workaround");
	}

	boot_primary_init_intc();
	init_vfp_nsec();
	if (!IS_ENABLED(CFG_NS_VIRTUALIZATION)) {
		/*
		 * Unmask native interrupts during driver initcalls.
		 *
		 * NS-virtualization still uses the temporary stack also
		 * used for exception handling so it must still have native
		 * interrupts masked.
		 */
		thread_set_exceptions(thread_get_exceptions() &
				      ~THREAD_EXCP_NATIVE_INTR);
		init_tee_runtime();
	}

	if (!IS_ENABLED(CFG_WITH_PAGER))
		boot_mem_release_tmp_alloc();
}

void __weak boot_init_primary_final(void)
{
	if (!IS_ENABLED(CFG_NS_VIRTUALIZATION))
		call_driver_initcalls();

	call_finalcalls();

	IMSG("Primary CPU switching to normal world boot");

	/* Mask native interrupts before switching to the normal world */
	if (!IS_ENABLED(CFG_NS_VIRTUALIZATION))
		thread_set_exceptions(thread_get_exceptions() |
				      THREAD_EXCP_NATIVE_INTR);
}

static void init_secondary_helper(void)
{
	IMSG("Secondary CPU %zu initializing", get_core_pos());

	/*
	 * Mask asynchronous exceptions before switch to the thread vector
	 * as the thread handler requires those to be masked while
	 * executing with the temporary stack. The thread subsystem also
	 * asserts that the foreign interrupts are blocked when using most of
	 * its functions.
	 */
	thread_set_exceptions(THREAD_EXCP_ALL);

	secondary_init_cntfrq();
	thread_init_per_cpu();
	boot_secondary_init_intc();
	init_vfp_sec();
	init_vfp_nsec();

	IMSG("Secondary CPU %zu switching to normal world boot", get_core_pos());
}

/*
 * Note: this function is weak just to make it possible to exclude it from
 * the unpaged area so that it lies in the init area.
 */
void __weak boot_init_primary_early(void)
{
	unsigned long pageable_part = 0;
	struct transfer_list_entry *tl_e = NULL;

	if (IS_ENABLED(CFG_TRANSFER_LIST) && boot_arg_transfer_list) {
		/* map and save the TL */
		mapped_tl = transfer_list_map(boot_arg_transfer_list);
		if (!mapped_tl)
			panic("Failed to map transfer list");

		transfer_list_dump(mapped_tl);
		tl_e = transfer_list_find(mapped_tl, TL_TAG_OPTEE_PAGABLE_PART);
	}

	if (IS_ENABLED(CFG_WITH_PAGER)) {
		if (IS_ENABLED(CFG_TRANSFER_LIST) && tl_e)
			pageable_part =
				get_le64(transfer_list_entry_data(tl_e));
		else
			pageable_part = boot_arg_pageable_part;
	}

	init_primary(pageable_part);
}

static void boot_save_transfer_list(unsigned long zero_reg,
				    unsigned long transfer_list,
				    unsigned long fdt)
{
	struct transfer_list_header *tl = (void *)transfer_list;
	struct transfer_list_entry *tl_e = NULL;

	if (zero_reg != 0)
		panic("Incorrect transfer list register convention");

	if (!IS_ALIGNED_WITH_TYPE(transfer_list, struct transfer_list_header) ||
	    !IS_ALIGNED(transfer_list, TL_ALIGNMENT_FROM_ORDER(tl->alignment)))
		panic("Transfer list base address is not aligned");

	if (transfer_list_check_header(tl) == TL_OPS_NONE)
		panic("Invalid transfer list");

	tl_e = transfer_list_find(tl, TL_TAG_FDT);
	if (fdt != (unsigned long)transfer_list_entry_data(tl_e))
		panic("DT does not match to the DT entry of the TL");

	boot_arg_transfer_list = transfer_list;
}

#if defined(CFG_WITH_ARM_TRUSTED_FW)
unsigned long boot_cpu_on_handler(unsigned long a0 __maybe_unused,
				  unsigned long a1 __unused)
{
	init_secondary_helper();
	return 0;
}
#else
void boot_init_secondary(unsigned long nsec_entry __unused)
{
	init_secondary_helper();
}
#endif

#if defined(CFG_BOOT_SECONDARY_REQUEST)
void boot_set_core_ns_entry(size_t core_idx, uintptr_t entry,
			    uintptr_t context_id)
{
	ns_entry_contexts[core_idx].entry_point = entry;
	ns_entry_contexts[core_idx].context_id = context_id;
	dsb_ishst();
}

int boot_core_release(size_t core_idx, paddr_t entry)
{
	if (!core_idx || core_idx >= CFG_TEE_CORE_NB_CORE)
		return -1;

	ns_entry_contexts[core_idx].entry_point = entry;
	dmb();
	spin_table[core_idx] = 1;
	dsb();
	sev();

	return 0;
}

/*
 * spin until secondary boot request, then returns with
 * the secondary core entry address.
 */
struct ns_entry_context *boot_core_hpen(void)
{
#ifdef CFG_PSCI_ARM32
	return &ns_entry_contexts[get_core_pos()];
#else
	do {
		wfe();
	} while (!spin_table[get_core_pos()]);
	dmb();
	return &ns_entry_contexts[get_core_pos()];
#endif
}
#endif

#if defined(CFG_CORE_ASLR)
#if defined(CFG_DT)
unsigned long __weak get_aslr_seed(void)
{
	void *fdt = NULL;
	int rc = 0;
	const uint64_t *seed = NULL;
	int offs = 0;
	int len = 0;

	if (!IS_ENABLED(CFG_CORE_SEL2_SPMC))
		fdt = (void *)boot_arg_fdt;

	if (!fdt) {
		DMSG("No fdt");
		goto err;
	}

	rc = fdt_check_header(fdt);
	if (rc) {
		DMSG("Bad fdt: %d", rc);
		goto err;
	}

	offs =  fdt_path_offset(fdt, "/secure-chosen");
	if (offs < 0) {
		DMSG("Cannot find /secure-chosen");
		goto err;
	}
	seed = fdt_getprop(fdt, offs, "kaslr-seed", &len);
	if (!seed || len != sizeof(*seed)) {
		DMSG("Cannot find valid kaslr-seed");
		goto err;
	}

	return fdt64_to_cpu(fdt64_ld(seed));

err:
	/* Try platform implementation */
	return plat_get_aslr_seed();
}
#else /*!CFG_DT*/
unsigned long __weak get_aslr_seed(void)
{
	/* Try platform implementation */
	return plat_get_aslr_seed();
}
#endif /*!CFG_DT*/
#endif /*CFG_CORE_ASLR*/

static void *get_fdt_from_boot_info(struct ffa_boot_info_header_1_1 *hdr)
{
	struct ffa_boot_info_1_1 *desc = NULL;
	uint8_t content_fmt = 0;
	uint8_t name_fmt = 0;
	void *fdt = NULL;
	int ret = 0;

	if (hdr->signature != FFA_BOOT_INFO_SIGNATURE) {
		EMSG("Bad boot info signature %#"PRIx32, hdr->signature);
		panic();
	}
	if (hdr->version != FFA_BOOT_INFO_VERSION_1_1 &&
	    hdr->version != FFA_BOOT_INFO_VERSION_1_2) {
		EMSG("Bad boot info version %#"PRIx32, hdr->version);
		panic();
	}
	if (hdr->desc_count != 1) {
		EMSG("Bad boot info descriptor count %#"PRIx32,
		     hdr->desc_count);
		panic();
	}
	desc = (void *)((vaddr_t)hdr + hdr->desc_offset);
	name_fmt = desc->flags & FFA_BOOT_INFO_FLAG_NAME_FORMAT_MASK;
	if (name_fmt == FFA_BOOT_INFO_FLAG_NAME_FORMAT_STRING)
		DMSG("Boot info descriptor name \"%16s\"", desc->name);
	else if (name_fmt == FFA_BOOT_INFO_FLAG_NAME_FORMAT_UUID)
		DMSG("Boot info descriptor UUID %pUl", (void *)desc->name);
	else
		DMSG("Boot info descriptor: unknown name format %"PRIu8,
		     name_fmt);

	content_fmt = (desc->flags & FFA_BOOT_INFO_FLAG_CONTENT_FORMAT_MASK) >>
		      FFA_BOOT_INFO_FLAG_CONTENT_FORMAT_SHIFT;
	if (content_fmt != FFA_BOOT_INFO_FLAG_CONTENT_FORMAT_ADDR) {
		EMSG("Bad boot info content format %"PRIu8", expected %u (address)",
		     content_fmt, FFA_BOOT_INFO_FLAG_CONTENT_FORMAT_ADDR);
		panic();
	}

	fdt = (void *)(vaddr_t)desc->contents;
	ret = fdt_check_full(fdt, desc->size);
	if (ret < 0) {
		EMSG("Invalid Device Tree at %p: error %d", fdt, ret);
		panic();
	}
	return fdt;
}

static void get_sec_mem_from_manifest(void *fdt, paddr_t *base,
				      paddr_size_t *size)
{
	int ret = 0;
	uint64_t num = 0;

	ret = fdt_node_check_compatible(fdt, 0, "arm,ffa-manifest-1.0");
	if (ret < 0) {
		EMSG("Invalid FF-A manifest at %p: error %d", fdt, ret);
		panic();
	}
	ret = dt_getprop_as_number(fdt, 0, "load-address", &num);
	if (ret < 0) {
		EMSG("Can't read \"load-address\" from FF-A manifest at %p: error %d",
		     fdt, ret);
		panic();
	}
	*base = num;
	/* "mem-size" is currently an undocumented extension to the spec. */
	ret = dt_getprop_as_number(fdt, 0, "mem-size", &num);
	if (ret < 0) {
		EMSG("Can't read \"mem-size\" from FF-A manifest at %p: error %d",
		     fdt, ret);
		panic();
	}
	*size = num;
}

void __weak boot_save_args(unsigned long a0, unsigned long a1,
			   unsigned long a2, unsigned long a3,
			   unsigned long a4 __maybe_unused)
{
	/*
	 * Register use:
	 *
	 * Scenario A: Default arguments
	 * a0   - CFG_CORE_FFA=y && CFG_CORE_SEL2_SPMC=n:
	 *        if non-NULL holds the TOS FW config [1] address
	 *      - CFG_CORE_FFA=y &&
		  (CFG_CORE_SEL2_SPMC=y || CFG_CORE_EL3_SPMC=y):
	 *        address of FF-A Boot Information Blob
	 *      - CFG_CORE_FFA=n:
	 *        if non-NULL holds the pagable part address
	 * a1	- CFG_WITH_ARM_TRUSTED_FW=n (Armv7):
	 *	  Armv7 standard bootarg #1 (kept track of in entry_a32.S)
	 * a2   - CFG_CORE_SEL2_SPMC=n:
	 *        if non-NULL holds the system DTB address
	 *	- CFG_WITH_ARM_TRUSTED_FW=n (Armv7):
	 *	  Armv7 standard bootarg #2 (system DTB address, kept track
	 *	  of in entry_a32.S)
	 * a3	- Not used
	 * a4	- CFG_WITH_ARM_TRUSTED_FW=n:
	 *	  Non-secure entry address
	 *
	 * [1] A TF-A concept: TOS_FW_CONFIG - Trusted OS Firmware
	 * configuration file. Used by Trusted OS (BL32), that is, OP-TEE
	 * here. This is also called Manifest DT, related to the Manifest DT
	 * passed in the FF-A Boot Information Blob, but with a different
	 * compatible string.

	 * Scenario B: FW Handoff via Transfer List
	 * Note: FF-A and non-secure entry are not yet supported with
	 *       Transfer List
	 * a0	- DTB address or 0 (AArch64)
	 *	- must be 0 (AArch32)
	 * a1	- 1 << 32 | TRANSFER_LIST_SIGNATURE[0:31] (AArch64)
	 *	- 1 << 24 | TRANSFER_LIST_SIGNATURE[0:23] (AArch32)
	 * a2	- must be 0 (AArch64)
	 *	- DTB address or 0 (AArch32)
	 * a3	- Transfer list base address
	 * a4	- Not used
	 */

	if (IS_ENABLED(CFG_TRANSFER_LIST)) {
		if (IS_ENABLED(CFG_ARM64_core) &&
		    a1 == TL_HANDOFF_X1_VALUE(TL_REG_CONVENTION_VER)) {
			boot_save_transfer_list(a2, a3, a0);
			boot_arg_fdt = a0;
		} else if (IS_ENABLED(CFG_ARM32_core) &&
			   a1 == TL_HANDOFF_R1_VALUE(TL_REG_CONVENTION_VER)) {
			boot_save_transfer_list(a0, a3, a2);
			boot_arg_fdt = a2;
		}

		return;
	}

	if (!IS_ENABLED(CFG_CORE_SEL2_SPMC)) {
#if defined(CFG_DT_ADDR)
		boot_arg_fdt = CFG_DT_ADDR;
#else
		boot_arg_fdt = a2;
#endif
	}

	if (IS_ENABLED(CFG_CORE_FFA)) {
		size_t fdt_max_size = CFG_DTB_MAX_SIZE;
		void *fdt = NULL;

		if (IS_ENABLED(CFG_CORE_SEL2_SPMC) ||
		    IS_ENABLED(CFG_CORE_EL3_SPMC))
			fdt = get_fdt_from_boot_info((void *)a0);
		else
			fdt = (void *)a0;
		if (IS_ENABLED(CFG_CORE_SEL2_SPMC)) {
			paddr_size_t size = 0;
			paddr_t base = 0;

			if (IS_ENABLED(CFG_CORE_PHYS_RELOCATABLE)) {
				get_sec_mem_from_manifest(fdt, &base, &size);
				core_mmu_set_secure_memory(base, size);
			} else {
				core_mmu_get_secure_memory(&base, &size);
			}
			assert((unsigned long)fdt >= base);
			assert((unsigned long)fdt <= base + size);
			assert((unsigned long)fdt < VCORE_START_VA);
			fdt_max_size = VCORE_START_VA - (unsigned long)fdt;
		}
		init_manifest_dt(fdt, fdt_max_size);
	} else {
		if (IS_ENABLED(CFG_WITH_PAGER)) {
#if defined(CFG_PAGEABLE_ADDR)
			boot_arg_pageable_part = CFG_PAGEABLE_ADDR;
#else
			boot_arg_pageable_part = a0;
#endif
		}
		if (!IS_ENABLED(CFG_WITH_ARM_TRUSTED_FW)) {
#if defined(CFG_NS_ENTRY_ADDR)
			boot_arg_nsec_entry = CFG_NS_ENTRY_ADDR;
#else
			boot_arg_nsec_entry = a4;
#endif
		}
	}
}

#if defined(CFG_TRANSFER_LIST)
static TEE_Result release_transfer_list(void)
{
	struct dt_descriptor *dt = get_external_dt_desc();

	if (!mapped_tl)
		return TEE_SUCCESS;

	if (dt) {
		int ret = 0;
		struct transfer_list_entry *tl_e = NULL;

		/*
		 * Pack the DTB and update the transfer list before un-mapping
		 */
		ret = fdt_pack(dt->blob);
		if (ret < 0) {
			EMSG("Failed to pack Device Tree at 0x%" PRIxPA
			     ": error %d", virt_to_phys(dt->blob), ret);
			panic();
		}

		tl_e = transfer_list_find(mapped_tl, TL_TAG_FDT);
		assert(dt->blob == transfer_list_entry_data(tl_e));
		transfer_list_set_data_size(mapped_tl, tl_e,
					    fdt_totalsize(dt->blob));
		dt->blob = NULL;
	}

	transfer_list_unmap_sync(mapped_tl);
	mapped_tl = NULL;

	return TEE_SUCCESS;
}

boot_final(release_transfer_list);
#endif
