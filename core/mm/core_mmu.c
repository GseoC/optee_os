// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2016-2025 Linaro Limited
 * Copyright (c) 2014, STMicroelectronics International N.V.
 * Copyright (c) 2022, Arm Limited and Contributors. All rights reserved.
 */

#include <assert.h>
#include <config.h>
#include <kernel/boot.h>
#include <kernel/dt.h>
#include <kernel/linker.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <kernel/tee_l2cc_mutex.h>
#include <kernel/tee_misc.h>
#include <kernel/tlb_helpers.h>
#include <kernel/user_mode_ctx.h>
#include <kernel/virtualization.h>
#include <libfdt.h>
#include <memtag.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <mm/mobj.h>
#include <mm/pgt_cache.h>
#include <mm/phys_mem.h>
#include <mm/tee_pager.h>
#include <mm/vm.h>
#include <platform_config.h>
#include <stdalign.h>
#include <string.h>
#include <trace.h>
#include <util.h>

#ifndef DEBUG_XLAT_TABLE
#define DEBUG_XLAT_TABLE 0
#endif

#define SHM_VASPACE_SIZE	(1024 * 1024 * 32)

/* Virtual memory pool for core mappings */
tee_mm_pool_t core_virt_mem_pool;

/* Virtual memory pool for shared memory mappings */
tee_mm_pool_t core_virt_shm_pool;

#ifdef CFG_CORE_PHYS_RELOCATABLE
unsigned long core_mmu_tee_load_pa __nex_bss;
#else
const unsigned long core_mmu_tee_load_pa = TEE_LOAD_ADDR;
#endif

/*
 * These variables are initialized before .bss is cleared. To avoid
 * resetting them when .bss is cleared we're storing them in .data instead,
 * even if they initially are zero.
 */

#ifdef CFG_CORE_RESERVED_SHM
/* Default NSec shared memory allocated from NSec world */
unsigned long default_nsec_shm_size __nex_bss;
unsigned long default_nsec_shm_paddr __nex_bss;
#endif

static struct memory_map static_memory_map __nex_bss;
void (*memory_map_realloc_func)(struct memory_map *mem_map) __nex_bss;

/* Offset of the first TEE RAM mapping from start of secure RAM */
static size_t tee_ram_initial_offs __nex_bss;

/* Define the platform's memory layout. */
struct memaccess_area {
	paddr_t paddr;
	size_t size;
};

#define MEMACCESS_AREA(a, s) { .paddr = a, .size = s }

static struct memaccess_area secure_only[] __nex_data = {
#ifdef CFG_CORE_PHYS_RELOCATABLE
	MEMACCESS_AREA(0, 0),
#else
#ifdef TRUSTED_SRAM_BASE
	MEMACCESS_AREA(TRUSTED_SRAM_BASE, TRUSTED_SRAM_SIZE),
#endif
	MEMACCESS_AREA(TRUSTED_DRAM_BASE, TRUSTED_DRAM_SIZE),
#endif
};

static struct memaccess_area nsec_shared[] __nex_data = {
#ifdef CFG_CORE_RESERVED_SHM
	MEMACCESS_AREA(TEE_SHMEM_START, TEE_SHMEM_SIZE),
#endif
};

#if defined(CFG_SECURE_DATA_PATH)
static const char *tz_sdp_match = "linaro,secure-heap";
static struct memaccess_area sec_sdp;
#ifdef CFG_TEE_SDP_MEM_BASE
register_sdp_mem(CFG_TEE_SDP_MEM_BASE, CFG_TEE_SDP_MEM_SIZE);
#endif
#ifdef TEE_SDP_TEST_MEM_BASE
register_sdp_mem(TEE_SDP_TEST_MEM_BASE, TEE_SDP_TEST_MEM_SIZE);
#endif
#endif

#ifdef CFG_CORE_RESERVED_SHM
register_phys_mem(MEM_AREA_NSEC_SHM, TEE_SHMEM_START, TEE_SHMEM_SIZE);
#endif
static unsigned int mmu_spinlock;

static uint32_t mmu_lock(void)
{
	return cpu_spin_lock_xsave(&mmu_spinlock);
}

static void mmu_unlock(uint32_t exceptions)
{
	cpu_spin_unlock_xrestore(&mmu_spinlock, exceptions);
}

static void heap_realloc_memory_map(struct memory_map *mem_map)
{
	struct tee_mmap_region *m = NULL;
	struct tee_mmap_region *old = mem_map->map;
	size_t old_sz = sizeof(*old) * mem_map->alloc_count;
	size_t sz = old_sz + sizeof(*m);

	assert(nex_malloc_buffer_is_within_alloced(old, old_sz));
	m = nex_realloc(old, sz);
	if (!m)
		panic();
	mem_map->map = m;
	mem_map->alloc_count++;
}

static void boot_mem_realloc_memory_map(struct memory_map *mem_map)
{
	struct tee_mmap_region *m = NULL;
	struct tee_mmap_region *old = mem_map->map;
	size_t old_sz = sizeof(*old) * mem_map->alloc_count;
	size_t sz = old_sz * 2;

	m = boot_mem_alloc_tmp(sz, alignof(*m));
	memcpy(m, old, old_sz);
	mem_map->map = m;
	mem_map->alloc_count *= 2;
}

static void grow_mem_map(struct memory_map *mem_map)
{
	if (mem_map->count == mem_map->alloc_count) {
		if (!memory_map_realloc_func) {
			EMSG("Out of entries (%zu) in mem_map",
			     mem_map->alloc_count);
			panic();
		}
		memory_map_realloc_func(mem_map);
	}
	mem_map->count++;
}

void core_mmu_get_secure_memory(paddr_t *base, paddr_size_t *size)
{
	/*
	 * The first range is always used to cover OP-TEE core memory, but
	 * depending on configuration it may cover more than that.
	 */
	*base = secure_only[0].paddr;
	*size = secure_only[0].size;
}

void core_mmu_set_secure_memory(paddr_t base, size_t size)
{
#ifdef CFG_CORE_PHYS_RELOCATABLE
	static_assert(ARRAY_SIZE(secure_only) == 1);
#endif
	runtime_assert(IS_ENABLED(CFG_CORE_PHYS_RELOCATABLE));
	assert(!secure_only[0].size);
	assert(base && size);

	DMSG("Physical secure memory base %#"PRIxPA" size %#zx", base, size);
	secure_only[0].paddr = base;
	secure_only[0].size = size;
}

static struct memory_map *get_memory_map(void)
{
	if (IS_ENABLED(CFG_NS_VIRTUALIZATION)) {
		struct memory_map *map = virt_get_memory_map();

		if (map)
			return map;
	}

	return &static_memory_map;
}

static bool _pbuf_intersects(struct memaccess_area *a, size_t alen,
			     paddr_t pa, size_t size)
{
	size_t n;

	for (n = 0; n < alen; n++)
		if (core_is_buffer_intersect(pa, size, a[n].paddr, a[n].size))
			return true;
	return false;
}

#define pbuf_intersects(a, pa, size) \
	_pbuf_intersects((a), ARRAY_SIZE(a), (pa), (size))

static bool _pbuf_is_inside(struct memaccess_area *a, size_t alen,
			    paddr_t pa, size_t size)
{
	size_t n;

	for (n = 0; n < alen; n++)
		if (core_is_buffer_inside(pa, size, a[n].paddr, a[n].size))
			return true;
	return false;
}

#define pbuf_is_inside(a, pa, size) \
	_pbuf_is_inside((a), ARRAY_SIZE(a), (pa), (size))

static bool pa_is_in_map(struct tee_mmap_region *map, paddr_t pa, size_t len)
{
	paddr_t end_pa = 0;

	if (!map)
		return false;

	if (SUB_OVERFLOW(len, 1, &end_pa) || ADD_OVERFLOW(pa, end_pa, &end_pa))
		return false;

	return (pa >= map->pa && end_pa <= map->pa + map->size - 1);
}

static bool va_is_in_map(struct tee_mmap_region *map, vaddr_t va)
{
	if (!map)
		return false;
	return (va >= map->va && va <= (map->va + map->size - 1));
}

/* check if target buffer fits in a core default map area */
static bool pbuf_inside_map_area(unsigned long p, size_t l,
				 struct tee_mmap_region *map)
{
	return core_is_buffer_inside(p, l, map->pa, map->size);
}

TEE_Result core_mmu_for_each_map(void *ptr,
				 TEE_Result (*fn)(struct tee_mmap_region *map,
						  void *ptr))
{
	struct memory_map *mem_map = get_memory_map();
	TEE_Result res = TEE_SUCCESS;
	size_t n = 0;

	for (n = 0; n < mem_map->count; n++) {
		res = fn(mem_map->map + n, ptr);
		if (res)
			return res;
	}

	return TEE_SUCCESS;
}

static struct tee_mmap_region *find_map_by_type(enum teecore_memtypes type)
{
	struct memory_map *mem_map = get_memory_map();
	size_t n = 0;

	for (n = 0; n < mem_map->count; n++) {
		if (mem_map->map[n].type == type)
			return mem_map->map + n;
	}
	return NULL;
}

static struct tee_mmap_region *
find_map_by_type_and_pa(enum teecore_memtypes type, paddr_t pa, size_t len)
{
	struct memory_map *mem_map = get_memory_map();
	size_t n = 0;

	for (n = 0; n < mem_map->count; n++) {
		if (mem_map->map[n].type != type)
			continue;
		if (pa_is_in_map(mem_map->map + n, pa, len))
			return mem_map->map + n;
	}
	return NULL;
}

static struct tee_mmap_region *find_map_by_va(void *va)
{
	struct memory_map *mem_map = get_memory_map();
	vaddr_t a = (vaddr_t)va;
	size_t n = 0;

	for (n = 0; n < mem_map->count; n++) {
		if (a >= mem_map->map[n].va &&
		    a <= (mem_map->map[n].va - 1 + mem_map->map[n].size))
			return mem_map->map + n;
	}

	return NULL;
}

static struct tee_mmap_region *find_map_by_pa(unsigned long pa)
{
	struct memory_map *mem_map = get_memory_map();
	size_t n = 0;

	for (n = 0; n < mem_map->count; n++) {
		/* Skip unmapped regions */
		if ((mem_map->map[n].attr & TEE_MATTR_VALID_BLOCK) &&
		    pa >= mem_map->map[n].pa &&
		    pa <= (mem_map->map[n].pa - 1 + mem_map->map[n].size))
			return mem_map->map + n;
	}

	return NULL;
}

#if defined(CFG_SECURE_DATA_PATH)
static bool dtb_get_sdp_region(void)
{
	void *fdt = NULL;
	int node = 0;
	int tmp_node = 0;
	paddr_t tmp_addr = 0;
	size_t tmp_size = 0;

	if (!IS_ENABLED(CFG_EMBED_DTB))
		return false;

	fdt = get_embedded_dt();
	if (!fdt)
		panic("No DTB found");

	node = fdt_node_offset_by_compatible(fdt, -1, tz_sdp_match);
	if (node < 0) {
		DMSG("No %s compatible node found", tz_sdp_match);
		return false;
	}
	tmp_node = node;
	while (tmp_node >= 0) {
		tmp_node = fdt_node_offset_by_compatible(fdt, tmp_node,
							 tz_sdp_match);
		if (tmp_node >= 0)
			DMSG("Ignore SDP pool node %s, supports only 1 node",
			     fdt_get_name(fdt, tmp_node, NULL));
	}

	if (fdt_reg_info(fdt, node, &tmp_addr, &tmp_size)) {
		EMSG("%s: Unable to get base addr or size from DT",
		     tz_sdp_match);
		return false;
	}

	sec_sdp.paddr = tmp_addr;
	sec_sdp.size = tmp_size;

	return true;
}
#endif

#if defined(CFG_CORE_DYN_SHM) || defined(CFG_SECURE_DATA_PATH)
static bool pbuf_is_special_mem(paddr_t pbuf, size_t len,
				const struct core_mmu_phys_mem *start,
				const struct core_mmu_phys_mem *end)
{
	const struct core_mmu_phys_mem *mem;

	for (mem = start; mem < end; mem++) {
		if (core_is_buffer_inside(pbuf, len, mem->addr, mem->size))
			return true;
	}

	return false;
}
#endif

#ifdef CFG_CORE_DYN_SHM
static void carve_out_phys_mem(struct core_mmu_phys_mem **mem, size_t *nelems,
			       paddr_t pa, size_t size)
{
	struct core_mmu_phys_mem *m = *mem;
	size_t n = 0;

	while (n < *nelems) {
		if (!core_is_buffer_intersect(pa, size, m[n].addr, m[n].size)) {
			n++;
			continue;
		}

		if (core_is_buffer_inside(m[n].addr, m[n].size, pa, size)) {
			/* m[n] is completely covered by pa:size */
			rem_array_elem(m, *nelems, sizeof(*m), n);
			(*nelems)--;
			m = nex_realloc(m, sizeof(*m) * *nelems);
			if (!m)
				panic();
			*mem = m;
			continue;
		}

		if (pa > m[n].addr &&
		    pa + size - 1 < m[n].addr + m[n].size - 1) {
			/*
			 * pa:size is strictly inside m[n] range so split
			 * m[n] entry.
			 */
			m = nex_realloc(m, sizeof(*m) * (*nelems + 1));
			if (!m)
				panic();
			*mem = m;
			(*nelems)++;
			ins_array_elem(m, *nelems, sizeof(*m), n + 1, NULL);
			m[n + 1].addr = pa + size;
			m[n + 1].size = m[n].addr + m[n].size - pa - size;
			m[n].size = pa - m[n].addr;
			n++;
		} else if (pa <= m[n].addr) {
			/*
			 * pa:size is overlapping (possibly partially) at the
			 * beginning of m[n].
			 */
			m[n].size = m[n].addr + m[n].size - pa - size;
			m[n].addr = pa + size;
		} else {
			/*
			 * pa:size is overlapping (possibly partially) at
			 * the end of m[n].
			 */
			m[n].size = pa - m[n].addr;
		}
		n++;
	}
}

static void check_phys_mem_is_outside(struct core_mmu_phys_mem *start,
				      size_t nelems,
				      struct tee_mmap_region *map)
{
	size_t n;

	for (n = 0; n < nelems; n++) {
		if (!core_is_buffer_outside(start[n].addr, start[n].size,
					    map->pa, map->size)) {
			EMSG("Non-sec mem (%#" PRIxPA ":%#" PRIxPASZ
			     ") overlaps map (type %d %#" PRIxPA ":%#zx)",
			     start[n].addr, start[n].size,
			     map->type, map->pa, map->size);
			panic();
		}
	}
}

static const struct core_mmu_phys_mem *discovered_nsec_ddr_start __nex_bss;
static size_t discovered_nsec_ddr_nelems __nex_bss;

static int cmp_pmem_by_addr(const void *a, const void *b)
{
	const struct core_mmu_phys_mem *pmem_a = a;
	const struct core_mmu_phys_mem *pmem_b = b;

	return CMP_TRILEAN(pmem_a->addr, pmem_b->addr);
}

void core_mmu_set_discovered_nsec_ddr(struct core_mmu_phys_mem *start,
				      size_t nelems)
{
	struct core_mmu_phys_mem *m = start;
	size_t num_elems = nelems;
	struct memory_map *mem_map = &static_memory_map;
	const struct core_mmu_phys_mem __maybe_unused *pmem;
	size_t n = 0;

	assert(!discovered_nsec_ddr_start);
	assert(m && num_elems);

	qsort(m, num_elems, sizeof(*m), cmp_pmem_by_addr);

	/*
	 * Non-secure shared memory and also secure data
	 * path memory are supposed to reside inside
	 * non-secure memory. Since NSEC_SHM and SDP_MEM
	 * are used for a specific purpose make holes for
	 * those memory in the normal non-secure memory.
	 *
	 * This has to be done since for instance QEMU
	 * isn't aware of which memory range in the
	 * non-secure memory is used for NSEC_SHM.
	 */

#ifdef CFG_SECURE_DATA_PATH
	if (dtb_get_sdp_region())
		carve_out_phys_mem(&m, &num_elems, sec_sdp.paddr, sec_sdp.size);

	for (pmem = phys_sdp_mem_begin; pmem < phys_sdp_mem_end; pmem++)
		carve_out_phys_mem(&m, &num_elems, pmem->addr, pmem->size);
#endif

	for (n = 0; n < ARRAY_SIZE(secure_only); n++)
		carve_out_phys_mem(&m, &num_elems, secure_only[n].paddr,
				   secure_only[n].size);

	for  (n = 0; n < mem_map->count; n++) {
		switch (mem_map->map[n].type) {
		case MEM_AREA_NSEC_SHM:
			carve_out_phys_mem(&m, &num_elems, mem_map->map[n].pa,
					   mem_map->map[n].size);
			break;
		case MEM_AREA_EXT_DT:
		case MEM_AREA_MANIFEST_DT:
		case MEM_AREA_RAM_NSEC:
		case MEM_AREA_RES_VASPACE:
		case MEM_AREA_SHM_VASPACE:
		case MEM_AREA_TS_VASPACE:
		case MEM_AREA_PAGER_VASPACE:
		case MEM_AREA_NEX_DYN_VASPACE:
		case MEM_AREA_TEE_DYN_VASPACE:
			break;
		default:
			check_phys_mem_is_outside(m, num_elems,
						  mem_map->map + n);
		}
	}

	discovered_nsec_ddr_start = m;
	discovered_nsec_ddr_nelems = num_elems;

	DMSG("Non-secure RAM:");
	for (n = 0; n < num_elems; n++)
		DMSG("%zu: pa %#"PRIxPA"..%#"PRIxPA" sz %#"PRIxPASZ,
		     n, m[n].addr, m[n].addr + m[n].size - 1, m[n].size);

	if (!core_mmu_check_end_pa(m[num_elems - 1].addr,
				   m[num_elems - 1].size))
		panic();
}

static bool get_discovered_nsec_ddr(const struct core_mmu_phys_mem **start,
				    const struct core_mmu_phys_mem **end)
{
	if (!discovered_nsec_ddr_start)
		return false;

	*start = discovered_nsec_ddr_start;
	*end = discovered_nsec_ddr_start + discovered_nsec_ddr_nelems;

	return true;
}

static bool pbuf_is_nsec_ddr(paddr_t pbuf, size_t len)
{
	const struct core_mmu_phys_mem *start;
	const struct core_mmu_phys_mem *end;

	if (!get_discovered_nsec_ddr(&start, &end))
		return false;

	return pbuf_is_special_mem(pbuf, len, start, end);
}

bool core_mmu_nsec_ddr_is_defined(void)
{
	const struct core_mmu_phys_mem *start;
	const struct core_mmu_phys_mem *end;

	if (!get_discovered_nsec_ddr(&start, &end))
		return false;

	return start != end;
}
#else
static bool pbuf_is_nsec_ddr(paddr_t pbuf __unused, size_t len __unused)
{
	return false;
}
#endif /*CFG_CORE_DYN_SHM*/

#define MSG_MEM_INSTERSECT(pa1, sz1, pa2, sz2) \
	EMSG("[%" PRIxPA " %" PRIx64 "] intersects [%" PRIxPA " %" PRIx64 "]", \
			pa1, (uint64_t)pa1 + (sz1), pa2, (uint64_t)pa2 + (sz2))

#ifdef CFG_SECURE_DATA_PATH
static bool pbuf_is_sdp_mem(paddr_t pbuf, size_t len)
{
	bool is_sdp_mem = false;

	if (sec_sdp.size)
		is_sdp_mem = core_is_buffer_inside(pbuf, len, sec_sdp.paddr,
						   sec_sdp.size);

	if (!is_sdp_mem)
		is_sdp_mem = pbuf_is_special_mem(pbuf, len, phys_sdp_mem_begin,
						 phys_sdp_mem_end);

	return is_sdp_mem;
}

static struct mobj *core_sdp_mem_alloc_mobj(paddr_t pa, size_t size)
{
	struct mobj *mobj = mobj_phys_alloc(pa, size, TEE_MATTR_MEM_TYPE_CACHED,
					    CORE_MEM_SDP_MEM);

	if (!mobj)
		panic("can't create SDP physical memory object");

	return mobj;
}

struct mobj **core_sdp_mem_create_mobjs(void)
{
	const struct core_mmu_phys_mem *mem = NULL;
	struct mobj **mobj_base = NULL;
	struct mobj **mobj = NULL;
	int cnt = phys_sdp_mem_end - phys_sdp_mem_begin;

	if (sec_sdp.size)
		cnt++;

	/* SDP mobjs table must end with a NULL entry */
	mobj_base = calloc(cnt + 1, sizeof(struct mobj *));
	if (!mobj_base)
		panic("Out of memory");

	mobj = mobj_base;

	for (mem = phys_sdp_mem_begin; mem < phys_sdp_mem_end; mem++, mobj++)
		*mobj = core_sdp_mem_alloc_mobj(mem->addr, mem->size);

	if (sec_sdp.size)
		*mobj = core_sdp_mem_alloc_mobj(sec_sdp.paddr, sec_sdp.size);

	return mobj_base;
}

#else /* CFG_SECURE_DATA_PATH */
static bool pbuf_is_sdp_mem(paddr_t pbuf __unused, size_t len __unused)
{
	return false;
}

#endif /* CFG_SECURE_DATA_PATH */

/* Check special memories comply with registered memories */
static void verify_special_mem_areas(struct memory_map *mem_map,
				     const struct core_mmu_phys_mem *start,
				     const struct core_mmu_phys_mem *end,
				     const char *area_name __maybe_unused)
{
	const struct core_mmu_phys_mem *mem = NULL;
	const struct core_mmu_phys_mem *mem2 = NULL;
	size_t n = 0;

	if (start == end) {
		DMSG("No %s memory area defined", area_name);
		return;
	}

	for (mem = start; mem < end; mem++)
		DMSG("%s memory [%" PRIxPA " %" PRIx64 "]",
		     area_name, mem->addr, (uint64_t)mem->addr + mem->size);

	/* Check memories do not intersect each other */
	for (mem = start; mem + 1 < end; mem++) {
		for (mem2 = mem + 1; mem2 < end; mem2++) {
			if (core_is_buffer_intersect(mem2->addr, mem2->size,
						     mem->addr, mem->size)) {
				MSG_MEM_INSTERSECT(mem2->addr, mem2->size,
						   mem->addr, mem->size);
				panic("Special memory intersection");
			}
		}
	}

	/*
	 * Check memories do not intersect any mapped memory.
	 * This is called before reserved VA space is loaded in mem_map.
	 */
	for (mem = start; mem < end; mem++) {
		for (n = 0; n < mem_map->count; n++) {
#ifdef TEE_SDP_TEST_MEM_BASE
			/*
			 * Ignore MEM_AREA_SEC_RAM_OVERALL since it covers
			 * TEE_SDP_TEST_MEM too.
			 */
			if (mem->addr == TEE_SDP_TEST_MEM_BASE &&
			    mem->size == TEE_SDP_TEST_MEM_SIZE &&
			    mem_map->map[n].type == MEM_AREA_SEC_RAM_OVERALL)
				continue;
#endif
			if (core_is_buffer_intersect(mem->addr, mem->size,
						     mem_map->map[n].pa,
						     mem_map->map[n].size)) {
				MSG_MEM_INSTERSECT(mem->addr, mem->size,
						   mem_map->map[n].pa,
						   mem_map->map[n].size);
				panic("Special memory intersection");
			}
		}
	}
}

static void merge_mmaps(struct tee_mmap_region *dst,
			const struct tee_mmap_region *src)
{
	paddr_t end_pa = MAX(dst->pa + dst->size - 1, src->pa + src->size - 1);
	paddr_t pa = MIN(dst->pa, src->pa);

	DMSG("Merging %#"PRIxPA"..%#"PRIxPA" and %#"PRIxPA"..%#"PRIxPA,
	     dst->pa, dst->pa + dst->size - 1, src->pa,
	     src->pa + src->size - 1);
	dst->pa = pa;
	dst->size = end_pa - pa + 1;
}

static bool mmaps_are_mergeable(const struct tee_mmap_region *r1,
				const struct tee_mmap_region *r2)
{
	if (r1->type != r2->type)
		return false;

	if (r1->pa == r2->pa)
		return true;

	if (r1->pa < r2->pa)
		return r1->pa + r1->size >= r2->pa;
	else
		return r2->pa + r2->size >= r1->pa;
}

static void add_phys_mem(struct memory_map *mem_map,
			 const char *mem_name __maybe_unused,
			 enum teecore_memtypes mem_type,
			 paddr_t mem_addr, paddr_size_t mem_size)
{
	size_t n = 0;
	const struct tee_mmap_region m0 = {
		.type = mem_type,
		.pa = mem_addr,
		.size = mem_size,
	};

	if (!mem_size)	/* Discard null size entries */
		return;

	/*
	 * If some ranges of memory of the same type do overlap
	 * each others they are coalesced into one entry. To help this
	 * added entries are sorted by increasing physical.
	 *
	 * Note that it's valid to have the same physical memory as several
	 * different memory types, for instance the same device memory
	 * mapped as both secure and non-secure. This will probably not
	 * happen often in practice.
	 */
	DMSG("%s type %s 0x%08" PRIxPA " size 0x%08" PRIxPASZ,
	     mem_name, teecore_memtype_name(mem_type), mem_addr, mem_size);
	for  (n = 0; n < mem_map->count; n++) {
		if (mmaps_are_mergeable(mem_map->map + n, &m0)) {
			merge_mmaps(mem_map->map + n, &m0);
			/*
			 * The merged result might be mergeable with the
			 * next or previous entry.
			 */
			if (n + 1 < mem_map->count &&
			    mmaps_are_mergeable(mem_map->map + n,
						mem_map->map + n + 1)) {
				merge_mmaps(mem_map->map + n,
					    mem_map->map + n + 1);
				rem_array_elem(mem_map->map, mem_map->count,
					       sizeof(*mem_map->map), n + 1);
				mem_map->count--;
			}
			if (n > 0 && mmaps_are_mergeable(mem_map->map + n - 1,
							 mem_map->map + n)) {
				merge_mmaps(mem_map->map + n - 1,
					    mem_map->map + n);
				rem_array_elem(mem_map->map, mem_map->count,
					       sizeof(*mem_map->map), n);
				mem_map->count--;
			}
			return;
		}
		if (mem_type < mem_map->map[n].type ||
		    (mem_type == mem_map->map[n].type &&
		     mem_addr < mem_map->map[n].pa))
			break; /* found the spot where to insert this memory */
	}

	grow_mem_map(mem_map);
	ins_array_elem(mem_map->map, mem_map->count, sizeof(*mem_map->map),
		       n, &m0);
}

static void add_va_space(struct memory_map *mem_map,
			 enum teecore_memtypes type, size_t size)
{
	size_t n = 0;

	DMSG("type %s size 0x%08zx", teecore_memtype_name(type), size);
	for  (n = 0; n < mem_map->count; n++) {
		if (type < mem_map->map[n].type)
			break;
	}

	grow_mem_map(mem_map);
	ins_array_elem(mem_map->map, mem_map->count, sizeof(*mem_map->map),
		       n, NULL);
	mem_map->map[n] = (struct tee_mmap_region){
		.type = type,
		.size = size,
	};
}

uint32_t core_mmu_type_to_attr(enum teecore_memtypes t)
{
	const uint32_t attr = TEE_MATTR_VALID_BLOCK;
	const uint32_t tagged = TEE_MATTR_MEM_TYPE_TAGGED <<
				TEE_MATTR_MEM_TYPE_SHIFT;
	const uint32_t cached = TEE_MATTR_MEM_TYPE_CACHED <<
				TEE_MATTR_MEM_TYPE_SHIFT;
	const uint32_t noncache = TEE_MATTR_MEM_TYPE_DEV <<
				  TEE_MATTR_MEM_TYPE_SHIFT;

	switch (t) {
	case MEM_AREA_TEE_RAM:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRWX | tagged;
	case MEM_AREA_TEE_RAM_RX:
	case MEM_AREA_INIT_RAM_RX:
	case MEM_AREA_IDENTITY_MAP_RX:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRX | tagged;
	case MEM_AREA_TEE_RAM_RO:
	case MEM_AREA_INIT_RAM_RO:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PR | tagged;
	case MEM_AREA_TEE_RAM_RW:
	case MEM_AREA_NEX_RAM_RO: /* This has to be r/w during init runtime */
	case MEM_AREA_NEX_RAM_RW:
	case MEM_AREA_NEX_DYN_VASPACE:
	case MEM_AREA_TEE_DYN_VASPACE:
	case MEM_AREA_TEE_ASAN:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRW | tagged;
	case MEM_AREA_TEE_COHERENT:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRWX | noncache;
	case MEM_AREA_NSEC_SHM:
	case MEM_AREA_NEX_NSEC_SHM:
		return attr | TEE_MATTR_PRW | cached;
	case MEM_AREA_MANIFEST_DT:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PR | cached;
	case MEM_AREA_TRANSFER_LIST:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRW | cached;
	case MEM_AREA_EXT_DT:
		/*
		 * If CFG_MAP_EXT_DT_SECURE is enabled map the external device
		 * tree as secure non-cached memory, otherwise, fall back to
		 * non-secure mapping.
		 */
		if (IS_ENABLED(CFG_MAP_EXT_DT_SECURE))
			return attr | TEE_MATTR_SECURE | TEE_MATTR_PRW |
			       noncache;
		fallthrough;
	case MEM_AREA_IO_NSEC:
		return attr | TEE_MATTR_PRW | noncache;
	case MEM_AREA_IO_SEC:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRW | noncache;
	case MEM_AREA_RAM_NSEC:
		return attr | TEE_MATTR_PRW | cached;
	case MEM_AREA_RAM_SEC:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRW | cached;
	case MEM_AREA_SEC_RAM_OVERALL:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PRW | tagged;
	case MEM_AREA_ROM_SEC:
		return attr | TEE_MATTR_SECURE | TEE_MATTR_PR | cached;
	case MEM_AREA_RES_VASPACE:
	case MEM_AREA_SHM_VASPACE:
		return 0;
	case MEM_AREA_PAGER_VASPACE:
		return TEE_MATTR_SECURE;
	default:
		panic("invalid type");
	}
}

static bool __maybe_unused map_is_tee_ram(const struct tee_mmap_region *mm)
{
	switch (mm->type) {
	case MEM_AREA_TEE_RAM:
	case MEM_AREA_TEE_RAM_RX:
	case MEM_AREA_TEE_RAM_RO:
	case MEM_AREA_TEE_RAM_RW:
	case MEM_AREA_INIT_RAM_RX:
	case MEM_AREA_INIT_RAM_RO:
	case MEM_AREA_NEX_RAM_RW:
	case MEM_AREA_NEX_RAM_RO:
	case MEM_AREA_TEE_ASAN:
		return true;
	default:
		return false;
	}
}

static bool __maybe_unused map_is_secure(const struct tee_mmap_region *mm)
{
	return !!(core_mmu_type_to_attr(mm->type) & TEE_MATTR_SECURE);
}

static bool __maybe_unused map_is_pgdir(const struct tee_mmap_region *mm)
{
	return mm->region_size == CORE_MMU_PGDIR_SIZE;
}

static int cmp_mmap_by_lower_va(const void *a, const void *b)
{
	const struct tee_mmap_region *mm_a = a;
	const struct tee_mmap_region *mm_b = b;

	return CMP_TRILEAN(mm_a->va, mm_b->va);
}

static void dump_mmap_table(struct memory_map *mem_map)
{
	size_t n = 0;

	for (n = 0; n < mem_map->count; n++) {
		struct tee_mmap_region *map __maybe_unused = mem_map->map + n;

		DMSG("type %-12s va 0x%08" PRIxVA "..0x%08" PRIxVA
		     " pa 0x%08" PRIxPA "..0x%08" PRIxPA " size 0x%08zx (%s)",
		     teecore_memtype_name(map->type), map->va,
		     map->va + map->size - 1, map->pa,
		     (paddr_t)(map->pa + map->size - 1), map->size,
		     map->region_size == SMALL_PAGE_SIZE ? "smallpg" : "pgdir");
	}
}

#if DEBUG_XLAT_TABLE

static void dump_xlat_table(vaddr_t va, unsigned int level)
{
	struct core_mmu_table_info tbl_info;
	unsigned int idx = 0;
	paddr_t pa;
	uint32_t attr;

	core_mmu_find_table(NULL, va, level, &tbl_info);
	va = tbl_info.va_base;
	for (idx = 0; idx < tbl_info.num_entries; idx++) {
		core_mmu_get_entry(&tbl_info, idx, &pa, &attr);
		if (attr || level > CORE_MMU_BASE_TABLE_LEVEL) {
			const char *security_bit = "";

			if (core_mmu_entry_have_security_bit(attr)) {
				if (attr & TEE_MATTR_SECURE)
					security_bit = "S";
				else
					security_bit = "NS";
			}

			if (attr & TEE_MATTR_TABLE) {
				DMSG_RAW("%*s [LVL%d] VA:0x%010" PRIxVA
					" TBL:0x%010" PRIxPA " %s",
					level * 2, "", level, va, pa,
					security_bit);
				dump_xlat_table(va, level + 1);
			} else if (attr) {
				DMSG_RAW("%*s [LVL%d] VA:0x%010" PRIxVA
					" PA:0x%010" PRIxPA " %s-%s-%s-%s",
					level * 2, "", level, va, pa,
					mattr_is_cached(attr) ? "MEM" :
					"DEV",
					attr & TEE_MATTR_PW ? "RW" : "RO",
					attr & TEE_MATTR_PX ? "X " : "XN",
					security_bit);
			} else {
				DMSG_RAW("%*s [LVL%d] VA:0x%010" PRIxVA
					    " INVALID\n",
					    level * 2, "", level, va);
			}
		}
		va += BIT64(tbl_info.shift);
	}
}

#else

static void dump_xlat_table(vaddr_t va __unused, int level __unused)
{
}

#endif

/*
 * Reserves virtual memory space for pager usage.
 *
 * From the start of the first memory used by the link script +
 * TEE_RAM_VA_SIZE should be covered, either with a direct mapping or empty
 * mapping for pager usage. This adds translation tables as needed for the
 * pager to operate.
 */
static void add_pager_vaspace(struct memory_map *mem_map)
{
	paddr_t begin = 0;
	paddr_t end = 0;
	size_t size = 0;
	size_t pos = 0;
	size_t n = 0;


	for (n = 0; n < mem_map->count; n++) {
		if (map_is_tee_ram(mem_map->map + n)) {
			if (!begin)
				begin = mem_map->map[n].pa;
			pos = n + 1;
		}
	}

	end = mem_map->map[pos - 1].pa + mem_map->map[pos - 1].size;
	assert(end - begin < TEE_RAM_VA_SIZE);
	size = TEE_RAM_VA_SIZE - (end - begin);

	grow_mem_map(mem_map);
	ins_array_elem(mem_map->map, mem_map->count, sizeof(*mem_map->map),
		       n, NULL);
	mem_map->map[n] = (struct tee_mmap_region){
		.type = MEM_AREA_PAGER_VASPACE,
		.size = size,
		.region_size = SMALL_PAGE_SIZE,
		.attr = core_mmu_type_to_attr(MEM_AREA_PAGER_VASPACE),
	};
}

static void check_sec_nsec_mem_config(void)
{
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(secure_only); n++) {
		if (pbuf_intersects(nsec_shared, secure_only[n].paddr,
				    secure_only[n].size))
			panic("Invalid memory access config: sec/nsec");
	}
}

static void collect_device_mem_ranges(struct memory_map *mem_map)
{
	const char *compatible = "arm,ffa-manifest-device-regions";
	void *fdt = get_manifest_dt();
	const char *name = NULL;
	uint64_t page_count = 0;
	uint64_t base = 0;
	int subnode = 0;
	int node = 0;

	assert(fdt);

	node = fdt_node_offset_by_compatible(fdt, 0, compatible);
	if (node < 0)
		return;

	fdt_for_each_subnode(subnode, fdt, node) {
		name = fdt_get_name(fdt, subnode, NULL);
		if (!name)
			continue;

		if (dt_getprop_as_number(fdt, subnode, "base-address",
					 &base)) {
			EMSG("Mandatory field is missing: base-address");
			continue;
		}

		if (base & SMALL_PAGE_MASK) {
			EMSG("base-address is not page aligned");
			continue;
		}

		if (dt_getprop_as_number(fdt, subnode, "pages-count",
					 &page_count)) {
			EMSG("Mandatory field is missing: pages-count");
			continue;
		}

		add_phys_mem(mem_map, name, MEM_AREA_IO_SEC,
			     base, page_count * SMALL_PAGE_SIZE);
	}
}

static void collect_mem_ranges(struct memory_map *mem_map)
{
	const struct core_mmu_phys_mem *mem = NULL;
	vaddr_t ram_start = secure_only[0].paddr;
	size_t n = 0;

#define ADD_PHYS_MEM(_type, _addr, _size) \
		add_phys_mem(mem_map, #_addr, (_type), (_addr), (_size))

	if (IS_ENABLED(CFG_CORE_RWDATA_NOEXEC)) {
		paddr_t next_pa = 0;

		/*
		 * Read-only and read-execute physical memory areas must
		 * not be mapped by MEM_AREA_SEC_RAM_OVERALL, but all the
		 * read/write should.
		 */
		ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, ram_start,
			     VCORE_UNPG_RX_PA - ram_start);
		assert(VCORE_UNPG_RX_PA >= ram_start);
		tee_ram_initial_offs = VCORE_UNPG_RX_PA - ram_start;
		DMSG("tee_ram_initial_offs %#zx", tee_ram_initial_offs);
		ADD_PHYS_MEM(MEM_AREA_TEE_RAM_RX, VCORE_UNPG_RX_PA,
			     VCORE_UNPG_RX_SZ);
		ADD_PHYS_MEM(MEM_AREA_TEE_RAM_RO, VCORE_UNPG_RO_PA,
			     VCORE_UNPG_RO_SZ);

		if (IS_ENABLED(CFG_NS_VIRTUALIZATION)) {
			ADD_PHYS_MEM(MEM_AREA_NEX_RAM_RO, VCORE_UNPG_RW_PA,
				     VCORE_UNPG_RW_SZ);
			ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, VCORE_UNPG_RW_PA,
				     VCORE_UNPG_RW_SZ);

			ADD_PHYS_MEM(MEM_AREA_NEX_RAM_RW, VCORE_NEX_RW_PA,
				     VCORE_NEX_RW_SZ);
			ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, VCORE_NEX_RW_PA,
				     VCORE_NEX_RW_SZ);

			ADD_PHYS_MEM(MEM_AREA_NEX_RAM_RW, VCORE_FREE_PA,
				     VCORE_FREE_SZ);
			ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, VCORE_FREE_PA,
				     VCORE_FREE_SZ);
			next_pa = VCORE_FREE_PA + VCORE_FREE_SZ;
		} else {
			ADD_PHYS_MEM(MEM_AREA_TEE_RAM_RW, VCORE_UNPG_RW_PA,
				     VCORE_UNPG_RW_SZ);
			ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, VCORE_UNPG_RW_PA,
				     VCORE_UNPG_RW_SZ);

			ADD_PHYS_MEM(MEM_AREA_TEE_RAM_RW, VCORE_FREE_PA,
				     VCORE_FREE_SZ);
			ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, VCORE_FREE_PA,
				     VCORE_FREE_SZ);
			next_pa = VCORE_FREE_PA + VCORE_FREE_SZ;
		}

		if (IS_ENABLED(CFG_WITH_PAGER)) {
			paddr_t pa = 0;
			size_t sz = 0;

			ADD_PHYS_MEM(MEM_AREA_INIT_RAM_RX, VCORE_INIT_RX_PA,
				     VCORE_INIT_RX_SZ);
			ADD_PHYS_MEM(MEM_AREA_INIT_RAM_RO, VCORE_INIT_RO_PA,
				     VCORE_INIT_RO_SZ);
			/*
			 * Core init mapping shall cover up to end of the
			 * physical RAM.  This is required since the hash
			 * table is appended to the binary data after the
			 * firmware build sequence.
			 */
			pa = VCORE_INIT_RO_PA + VCORE_INIT_RO_SZ;
			sz = TEE_RAM_START + TEE_RAM_PH_SIZE - pa;
			ADD_PHYS_MEM(MEM_AREA_TEE_RAM, pa, sz);
		} else {
			ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, next_pa,
				     secure_only[0].paddr +
				     secure_only[0].size - next_pa);
		}
	} else {
		ADD_PHYS_MEM(MEM_AREA_TEE_RAM, TEE_RAM_START, TEE_RAM_PH_SIZE);
		ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, secure_only[n].paddr,
			     secure_only[0].size);
	}

	for (n = 1; n < ARRAY_SIZE(secure_only); n++)
		ADD_PHYS_MEM(MEM_AREA_SEC_RAM_OVERALL, secure_only[n].paddr,
			     secure_only[n].size);

	if (IS_ENABLED(CFG_CORE_SANITIZE_KADDRESS))
		ADD_PHYS_MEM(MEM_AREA_TEE_ASAN, ASAN_MAP_PA, ASAN_MAP_SZ);

#undef ADD_PHYS_MEM

	/* Collect device memory info from SP manifest */
	if (IS_ENABLED(CFG_CORE_SEL2_SPMC))
		collect_device_mem_ranges(mem_map);

	for (mem = phys_mem_map_begin; mem < phys_mem_map_end; mem++) {
		/* Only unmapped virtual range may have a null phys addr */
		assert(mem->addr || !core_mmu_type_to_attr(mem->type));

		add_phys_mem(mem_map, mem->name, mem->type,
			     mem->addr, mem->size);
	}

	if (IS_ENABLED(CFG_SECURE_DATA_PATH))
		verify_special_mem_areas(mem_map, phys_sdp_mem_begin,
					 phys_sdp_mem_end, "SDP");

	add_va_space(mem_map, MEM_AREA_RES_VASPACE, CFG_RESERVED_VASPACE_SIZE);
	add_va_space(mem_map, MEM_AREA_SHM_VASPACE, SHM_VASPACE_SIZE);
	if (IS_ENABLED(CFG_DYN_CONFIG)) {
		if (IS_ENABLED(CFG_NS_VIRTUALIZATION))
			add_va_space(mem_map, MEM_AREA_NEX_DYN_VASPACE,
				     ROUNDUP(CFG_NEX_DYN_VASPACE_SIZE,
					     CORE_MMU_PGDIR_SIZE));
		add_va_space(mem_map, MEM_AREA_TEE_DYN_VASPACE,
			     CFG_TEE_DYN_VASPACE_SIZE);
	}
}

static void assign_mem_granularity(struct memory_map *mem_map)
{
	size_t n = 0;

	/*
	 * Assign region sizes, note that MEM_AREA_TEE_RAM always uses
	 * SMALL_PAGE_SIZE.
	 */
	for  (n = 0; n < mem_map->count; n++) {
		paddr_t mask = mem_map->map[n].pa | mem_map->map[n].size;

		if (mask & SMALL_PAGE_MASK)
			panic("Impossible memory alignment");

		if (map_is_tee_ram(mem_map->map + n))
			mem_map->map[n].region_size = SMALL_PAGE_SIZE;
		else
			mem_map->map[n].region_size = CORE_MMU_PGDIR_SIZE;
	}
}

static bool place_tee_ram_at_top(paddr_t paddr)
{
	return paddr > BIT64(core_mmu_get_va_width()) / 2;
}

/*
 * MMU arch driver shall override this function if it helps
 * optimizing the memory footprint of the address translation tables.
 */
bool __weak core_mmu_prefer_tee_ram_at_top(paddr_t paddr)
{
	return place_tee_ram_at_top(paddr);
}

static bool assign_mem_va_dir(vaddr_t tee_ram_va, struct memory_map *mem_map,
			      bool tee_ram_at_top)
{
	struct tee_mmap_region *map = NULL;
	bool va_is_nex_shared = false;
	bool va_is_secure = true;
	vaddr_t va = 0;
	size_t n = 0;

	/*
	 * tee_ram_va might equals 0 when CFG_CORE_ASLR=y.
	 * 0 is by design an invalid va, so return false directly.
	 */
	if (!tee_ram_va)
		return false;

	/* Clear eventual previous assignments */
	for (n = 0; n < mem_map->count; n++)
		mem_map->map[n].va = 0;

	/*
	 * TEE RAM regions are always aligned with region_size.
	 *
	 * Note that MEM_AREA_PAGER_VASPACE also counts as TEE RAM here
	 * since it handles virtual memory which covers the part of the ELF
	 * that cannot fit directly into memory.
	 */
	va = tee_ram_va + tee_ram_initial_offs;
	for (n = 0; n < mem_map->count; n++) {
		map = mem_map->map + n;
		if (map_is_tee_ram(map) ||
		    map->type == MEM_AREA_PAGER_VASPACE) {
			assert(!(va & (map->region_size - 1)));
			assert(!(map->size & (map->region_size - 1)));
			map->va = va;
			if (ADD_OVERFLOW(va, map->size, &va))
				return false;
			if (!core_mmu_va_is_valid(va))
				return false;
		}
	}

	if (tee_ram_at_top) {
		/*
		 * Map non-tee ram regions at addresses lower than the tee
		 * ram region.
		 */
		va = tee_ram_va;
		for (n = 0; n < mem_map->count; n++) {
			map = mem_map->map + n;
			map->attr = core_mmu_type_to_attr(map->type);
			if (map->va)
				continue;

			if (!IS_ENABLED(CFG_WITH_LPAE) &&
			    va_is_secure != map_is_secure(map)) {
				va_is_secure = !va_is_secure;
				va = ROUNDDOWN(va, CORE_MMU_PGDIR_SIZE);
			} else if (va_is_nex_shared !=
				   core_mmu_type_is_nex_shared(map->type)) {
				va_is_nex_shared = !va_is_nex_shared;
				va = ROUNDDOWN(va, CORE_MMU_PGDIR_SIZE);
			}

			if (SUB_OVERFLOW(va, map->size, &va))
				return false;
			va = ROUNDDOWN2(va, map->region_size);
			/*
			 * Make sure that va is aligned with pa for
			 * efficient pgdir mapping. Basically pa &
			 * pgdir_mask should be == va & pgdir_mask
			 */
			if (map->size > 2 * CORE_MMU_PGDIR_SIZE) {
				if (SUB_OVERFLOW(va, CORE_MMU_PGDIR_SIZE, &va))
					return false;
				va += (map->pa - va) & CORE_MMU_PGDIR_MASK;
			}
			map->va = va;
		}
	} else {
		/*
		 * Map non-tee ram regions at addresses higher than the tee
		 * ram region.
		 */
		for (n = 0; n < mem_map->count; n++) {
			map = mem_map->map + n;
			map->attr = core_mmu_type_to_attr(map->type);
			if (map->va)
				continue;

			if (!IS_ENABLED(CFG_WITH_LPAE) &&
			    va_is_secure != map_is_secure(map)) {
				va_is_secure = !va_is_secure;
				if (ROUNDUP_OVERFLOW(va, CORE_MMU_PGDIR_SIZE,
						     &va))
					return false;
			} else if (va_is_nex_shared !=
				   core_mmu_type_is_nex_shared(map->type)) {
				va_is_nex_shared = !va_is_nex_shared;
				if (ROUNDUP_OVERFLOW(va, CORE_MMU_PGDIR_SIZE,
						     &va))
					return false;
			}

			if (ROUNDUP2_OVERFLOW(va, map->region_size, &va))
				return false;
			/*
			 * Make sure that va is aligned with pa for
			 * efficient pgdir mapping. Basically pa &
			 * pgdir_mask should be == va & pgdir_mask
			 */
			if (map->size > 2 * CORE_MMU_PGDIR_SIZE) {
				vaddr_t offs = (map->pa - va) &
					       CORE_MMU_PGDIR_MASK;

				if (ADD_OVERFLOW(va, offs, &va))
					return false;
			}

			map->va = va;
			if (ADD_OVERFLOW(va, map->size, &va))
				return false;
			if (!core_mmu_va_is_valid(va))
				return false;
		}
	}

	return true;
}

static bool assign_mem_va(vaddr_t tee_ram_va, struct memory_map *mem_map)
{
	bool tee_ram_at_top = place_tee_ram_at_top(tee_ram_va);

	/*
	 * Check that we're not overlapping with the user VA range.
	 */
	if (IS_ENABLED(CFG_WITH_LPAE)) {
		/*
		 * User VA range is supposed to be defined after these
		 * mappings have been established.
		 */
		assert(!core_mmu_user_va_range_is_defined());
	} else {
		vaddr_t user_va_base = 0;
		size_t user_va_size = 0;

		assert(core_mmu_user_va_range_is_defined());
		core_mmu_get_user_va_range(&user_va_base, &user_va_size);
		if (tee_ram_va < (user_va_base + user_va_size))
			return false;
	}

	if (IS_ENABLED(CFG_WITH_PAGER)) {
		bool prefered_dir = core_mmu_prefer_tee_ram_at_top(tee_ram_va);

		/* Try whole mapping covered by a single base xlat entry */
		if (prefered_dir != tee_ram_at_top &&
		    assign_mem_va_dir(tee_ram_va, mem_map, prefered_dir))
			return true;
	}

	return assign_mem_va_dir(tee_ram_va, mem_map, tee_ram_at_top);
}

static int cmp_init_mem_map(const void *a, const void *b)
{
	const struct tee_mmap_region *mm_a = a;
	const struct tee_mmap_region *mm_b = b;
	int rc = 0;

	rc = CMP_TRILEAN(mm_a->region_size, mm_b->region_size);
	if (!rc)
		rc = CMP_TRILEAN(mm_a->pa, mm_b->pa);
	/*
	 * 32bit MMU descriptors cannot mix secure and non-secure mapping in
	 * the same level2 table. Hence sort secure mapping from non-secure
	 * mapping.
	 */
	if (!rc && !IS_ENABLED(CFG_WITH_LPAE))
		rc = CMP_TRILEAN(map_is_secure(mm_a), map_is_secure(mm_b));

	/*
	 * Nexus mappings shared between partitions should not be mixed
	 * with other mappings in the same translation table. Hence sort
	 * nexus shared mappings from other mappings.
	 */
	if (!rc)
		rc = CMP_TRILEAN(core_mmu_type_is_nex_shared(mm_a->type),
				 core_mmu_type_is_nex_shared(mm_b->type));

	return rc;
}

static bool mem_map_add_id_map(struct memory_map *mem_map,
			       vaddr_t id_map_start, vaddr_t id_map_end)
{
	vaddr_t start = ROUNDDOWN(id_map_start, SMALL_PAGE_SIZE);
	vaddr_t end = ROUNDUP(id_map_end, SMALL_PAGE_SIZE);
	size_t len = end - start;
	size_t n = 0;


	for (n = 0; n < mem_map->count; n++)
		if (core_is_buffer_intersect(mem_map->map[n].va,
					     mem_map->map[n].size, start, len))
			return false;

	grow_mem_map(mem_map);
	mem_map->map[mem_map->count - 1] = (struct tee_mmap_region){
		.type = MEM_AREA_IDENTITY_MAP_RX,
		/*
		 * Could use CORE_MMU_PGDIR_SIZE to potentially save a
		 * translation table, at the increased risk of clashes with
		 * the rest of the memory map.
		 */
		.region_size = SMALL_PAGE_SIZE,
		.pa = start,
		.va = start,
		.size = len,
		.attr = core_mmu_type_to_attr(MEM_AREA_IDENTITY_MAP_RX),
	};

	return true;
}

static struct memory_map *init_mem_map(struct memory_map *mem_map,
				       unsigned long seed,
				       unsigned long *ret_offs)
{
	/*
	 * @id_map_start and @id_map_end describes a physical memory range
	 * that must be mapped Read-Only eXecutable at identical virtual
	 * addresses.
	 */
	vaddr_t id_map_start = (vaddr_t)__identity_map_init_start;
	vaddr_t id_map_end = (vaddr_t)__identity_map_init_end;
	vaddr_t start_addr = secure_only[0].paddr;
	unsigned long offs = 0;

	collect_mem_ranges(mem_map);
	assign_mem_granularity(mem_map);

	/*
	 * To ease mapping and lower use of xlat tables, sort mapping
	 * description moving small-page regions after the pgdir regions.
	 */
	qsort(mem_map->map, mem_map->count, sizeof(struct tee_mmap_region),
	      cmp_init_mem_map);

	if (IS_ENABLED(CFG_WITH_PAGER))
		add_pager_vaspace(mem_map);

	if (IS_ENABLED(CFG_CORE_ASLR) && seed) {
		vaddr_t ba = 0;
		size_t n = 0;

		for (n = 0; n < 3; n++) {
			ba = arch_aslr_base_addr(start_addr, seed, n);
			if (assign_mem_va(ba, mem_map) &&
			    mem_map_add_id_map(mem_map, id_map_start,
					       id_map_end)) {
				offs = ba - start_addr;
				DMSG("Mapping core at %#"PRIxVA" offs %#lx",
				     ba, offs);
				goto out;
			} else {
				DMSG("Failed to map core at %#"PRIxVA, ba);
			}
		}
		EMSG("Failed to map core with seed %#lx", seed);
	}

	if (!assign_mem_va(start_addr, mem_map))
		panic();

out:
	qsort(mem_map->map, mem_map->count, sizeof(struct tee_mmap_region),
	      cmp_mmap_by_lower_va);

	dump_mmap_table(mem_map);

	*ret_offs = offs;
	return mem_map;
}

static void check_mem_map(struct memory_map *mem_map)
{
	struct tee_mmap_region *m = NULL;
	size_t n = 0;

	for (n = 0; n < mem_map->count; n++) {
		m = mem_map->map + n;
		switch (m->type) {
		case MEM_AREA_TEE_RAM:
		case MEM_AREA_TEE_RAM_RX:
		case MEM_AREA_TEE_RAM_RO:
		case MEM_AREA_TEE_RAM_RW:
		case MEM_AREA_INIT_RAM_RX:
		case MEM_AREA_INIT_RAM_RO:
		case MEM_AREA_NEX_RAM_RW:
		case MEM_AREA_NEX_RAM_RO:
		case MEM_AREA_IDENTITY_MAP_RX:
			if (!pbuf_is_inside(secure_only, m->pa, m->size))
				panic("TEE_RAM can't fit in secure_only");
			break;
		case MEM_AREA_SEC_RAM_OVERALL:
			if (!pbuf_is_inside(secure_only, m->pa, m->size))
				panic("SEC_RAM_OVERALL can't fit in secure_only");
			break;
		case MEM_AREA_NSEC_SHM:
			if (!pbuf_is_inside(nsec_shared, m->pa, m->size))
				panic("NS_SHM can't fit in nsec_shared");
			break;
		case MEM_AREA_TEE_COHERENT:
		case MEM_AREA_TEE_ASAN:
		case MEM_AREA_IO_SEC:
		case MEM_AREA_IO_NSEC:
		case MEM_AREA_EXT_DT:
		case MEM_AREA_MANIFEST_DT:
		case MEM_AREA_TRANSFER_LIST:
		case MEM_AREA_RAM_SEC:
		case MEM_AREA_RAM_NSEC:
		case MEM_AREA_ROM_SEC:
		case MEM_AREA_RES_VASPACE:
		case MEM_AREA_SHM_VASPACE:
		case MEM_AREA_PAGER_VASPACE:
		case MEM_AREA_NEX_DYN_VASPACE:
		case MEM_AREA_TEE_DYN_VASPACE:
			break;
		default:
			EMSG("Uhandled memtype %d", m->type);
			panic();
		}
	}
}

/*
 * core_init_mmu_map() - init tee core default memory mapping
 *
 * This routine sets the static default TEE core mapping. If @seed is > 0
 * and configured with CFG_CORE_ASLR it will map tee core at a location
 * based on the seed and return the offset from the link address.
 *
 * If an error happened: core_init_mmu_map is expected to panic.
 *
 * Note: this function is weak just to make it possible to exclude it from
 * the unpaged area.
 */
void __weak core_init_mmu_map(unsigned long seed, struct core_mmu_config *cfg)
{
#ifndef CFG_NS_VIRTUALIZATION
	vaddr_t start = ROUNDDOWN((vaddr_t)__nozi_start, SMALL_PAGE_SIZE);
#else
	vaddr_t start = ROUNDDOWN((vaddr_t)__vcore_nex_rw_start,
				  SMALL_PAGE_SIZE);
#endif
#ifdef CFG_DYN_CONFIG
	vaddr_t len = ROUNDUP(VCORE_FREE_END_PA, SMALL_PAGE_SIZE) - start;
#else
	vaddr_t len = ROUNDUP((vaddr_t)__nozi_end, SMALL_PAGE_SIZE) - start;
#endif
	struct tee_mmap_region tmp_mmap_region = { };
	struct memory_map mem_map = { };
	unsigned long offs = 0;

	if (IS_ENABLED(CFG_CORE_PHYS_RELOCATABLE) &&
	    (core_mmu_tee_load_pa & SMALL_PAGE_MASK))
		panic("OP-TEE load address is not page aligned");

	check_sec_nsec_mem_config();

	mem_map.alloc_count = CFG_MMAP_REGIONS;
	mem_map.map = boot_mem_alloc_tmp(mem_map.alloc_count *
						sizeof(*mem_map.map),
					 alignof(*mem_map.map));
	memory_map_realloc_func = boot_mem_realloc_memory_map;

	static_memory_map = (struct memory_map){
		.map = &tmp_mmap_region,
		.alloc_count = 1,
		.count = 1,
	};
	/*
	 * Add a entry covering the translation tables which will be
	 * involved in some virt_to_phys() and phys_to_virt() conversions.
	 */
	static_memory_map.map[0] = (struct tee_mmap_region){
		.type = MEM_AREA_TEE_RAM,
		.region_size = SMALL_PAGE_SIZE,
		.pa = start,
		.va = start,
		.size = len,
		.attr = core_mmu_type_to_attr(MEM_AREA_IDENTITY_MAP_RX),
	};

	init_mem_map(&mem_map, seed, &offs);

	check_mem_map(&mem_map);
	core_init_mmu(&mem_map);
	dump_xlat_table(0x0, CORE_MMU_BASE_TABLE_LEVEL);
	core_init_mmu_regs(cfg);
	cfg->map_offset = offs;
	static_memory_map = mem_map;
	boot_mem_add_reloc(&static_memory_map.map);
}

void core_mmu_save_mem_map(void)
{
	size_t alloc_count = static_memory_map.count + 5;
	size_t elem_sz = sizeof(*static_memory_map.map);
	void *p = NULL;

	p = nex_calloc(alloc_count, elem_sz);
	if (!p)
		panic();
	memcpy(p, static_memory_map.map, static_memory_map.count * elem_sz);
	static_memory_map.map = p;
	static_memory_map.alloc_count = alloc_count;
	memory_map_realloc_func = heap_realloc_memory_map;
}

bool core_mmu_mattr_is_ok(uint32_t mattr)
{
	/*
	 * Keep in sync with core_mmu_lpae.c:mattr_to_desc and
	 * core_mmu_v7.c:mattr_to_texcb
	 */

	switch ((mattr >> TEE_MATTR_MEM_TYPE_SHIFT) & TEE_MATTR_MEM_TYPE_MASK) {
	case TEE_MATTR_MEM_TYPE_DEV:
	case TEE_MATTR_MEM_TYPE_STRONGLY_O:
	case TEE_MATTR_MEM_TYPE_CACHED:
	case TEE_MATTR_MEM_TYPE_TAGGED:
		return true;
	default:
		return false;
	}
}

/*
 * test attributes of target physical buffer
 *
 * Flags: pbuf_is(SECURE, NOT_SECURE, RAM, IOMEM, KEYVAULT).
 *
 */
bool core_pbuf_is(uint32_t attr, paddr_t pbuf, size_t len)
{
	struct tee_mmap_region *map;

	/* Empty buffers complies with anything */
	if (len == 0)
		return true;

	switch (attr) {
	case CORE_MEM_SEC:
		return pbuf_is_inside(secure_only, pbuf, len);
	case CORE_MEM_NON_SEC:
		return pbuf_is_inside(nsec_shared, pbuf, len) ||
			pbuf_is_nsec_ddr(pbuf, len);
	case CORE_MEM_TEE_RAM:
		return core_is_buffer_inside(pbuf, len, TEE_RAM_START,
							TEE_RAM_PH_SIZE);
#ifdef CFG_CORE_RESERVED_SHM
	case CORE_MEM_NSEC_SHM:
		return core_is_buffer_inside(pbuf, len, TEE_SHMEM_START,
							TEE_SHMEM_SIZE);
#endif
	case CORE_MEM_SDP_MEM:
		return pbuf_is_sdp_mem(pbuf, len);
	case CORE_MEM_CACHED:
		map = find_map_by_pa(pbuf);
		if (!map || !pbuf_inside_map_area(pbuf, len, map))
			return false;
		return mattr_is_cached(map->attr);
	default:
		return false;
	}
}

/* test attributes of target virtual buffer (in core mapping) */
bool core_vbuf_is(uint32_t attr, const void *vbuf, size_t len)
{
	paddr_t p;

	/* Empty buffers complies with anything */
	if (len == 0)
		return true;

	p = virt_to_phys((void *)vbuf);
	if (!p)
		return false;

	return core_pbuf_is(attr, p, len);
}

/* core_va2pa - teecore exported service */
static int __maybe_unused core_va2pa_helper(void *va, paddr_t *pa)
{
	struct tee_mmap_region *map;

	map = find_map_by_va(va);
	if (!va_is_in_map(map, (vaddr_t)va))
		return -1;

	/*
	 * We can calculate PA for static map. Virtual address ranges
	 * reserved to core dynamic mapping return a 'match' (return 0;)
	 * together with an invalid null physical address.
	 */
	if (map->pa)
		*pa = map->pa + (vaddr_t)va  - map->va;
	else
		*pa = 0;

	return 0;
}

static void *map_pa2va(struct tee_mmap_region *map, paddr_t pa, size_t len)
{
	if (!pa_is_in_map(map, pa, len))
		return NULL;

	return (void *)(vaddr_t)(map->va + pa - map->pa);
}

/*
 * teecore gets some memory area definitions
 */
void core_mmu_get_mem_by_type(enum teecore_memtypes type, vaddr_t *s,
			      vaddr_t *e)
{
	struct tee_mmap_region *map = find_map_by_type(type);

	if (map) {
		*s = map->va;
		*e = map->va + map->size;
	} else {
		*s = 0;
		*e = 0;
	}
}

enum teecore_memtypes core_mmu_get_type_by_pa(paddr_t pa)
{
	struct tee_mmap_region *map = find_map_by_pa(pa);

	if (!map)
		return MEM_AREA_MAXTYPE;
	return map->type;
}

void core_mmu_set_entry(struct core_mmu_table_info *tbl_info, unsigned int idx,
			paddr_t pa, uint32_t attr)
{
	assert(idx < tbl_info->num_entries);
	core_mmu_set_entry_primitive(tbl_info->table, tbl_info->level,
				     idx, pa, attr);
}

void core_mmu_get_entry(struct core_mmu_table_info *tbl_info, unsigned int idx,
			paddr_t *pa, uint32_t *attr)
{
	assert(idx < tbl_info->num_entries);
	core_mmu_get_entry_primitive(tbl_info->table, tbl_info->level,
				     idx, pa, attr);
}

static void clear_region(struct core_mmu_table_info *tbl_info,
			 struct tee_mmap_region *region)
{
	unsigned int end = 0;
	unsigned int idx = 0;

	/* va, len and pa should be block aligned */
	assert(!core_mmu_get_block_offset(tbl_info, region->va));
	assert(!core_mmu_get_block_offset(tbl_info, region->size));
	assert(!core_mmu_get_block_offset(tbl_info, region->pa));

	idx = core_mmu_va2idx(tbl_info, region->va);
	end = core_mmu_va2idx(tbl_info, region->va + region->size);

	while (idx < end) {
		core_mmu_set_entry(tbl_info, idx, 0, 0);
		idx++;
	}
}

static void set_region(struct core_mmu_table_info *tbl_info,
		       struct tee_mmap_region *region)
{
	unsigned int end;
	unsigned int idx;
	paddr_t pa;

	/* va, len and pa should be block aligned */
	assert(!core_mmu_get_block_offset(tbl_info, region->va));
	assert(!core_mmu_get_block_offset(tbl_info, region->size));
	assert(!core_mmu_get_block_offset(tbl_info, region->pa));

	idx = core_mmu_va2idx(tbl_info, region->va);
	end = core_mmu_va2idx(tbl_info, region->va + region->size);
	pa = region->pa;

	while (idx < end) {
		core_mmu_set_entry(tbl_info, idx, pa, region->attr);
		idx++;
		pa += BIT64(tbl_info->shift);
	}
}

static void set_pg_region(struct core_mmu_table_info *dir_info,
			  struct vm_region *region, struct pgt **pgt,
			  struct core_mmu_table_info *pg_info)
{
	struct tee_mmap_region r = {
		.va = region->va,
		.size = region->size,
		.attr = region->attr,
	};
	vaddr_t end = r.va + r.size;
	uint32_t pgt_attr = (r.attr & TEE_MATTR_SECURE) | TEE_MATTR_TABLE;

	while (r.va < end) {
		if (!pg_info->table ||
		    r.va >= (pg_info->va_base + CORE_MMU_PGDIR_SIZE)) {
			/*
			 * We're assigning a new translation table.
			 */
			unsigned int idx;

			/* Virtual addresses must grow */
			assert(r.va > pg_info->va_base);

			idx = core_mmu_va2idx(dir_info, r.va);
			pg_info->va_base = core_mmu_idx2va(dir_info, idx);

			/*
			 * Advance pgt to va_base, note that we may need to
			 * skip multiple page tables if there are large
			 * holes in the vm map.
			 */
			while ((*pgt)->vabase < pg_info->va_base) {
				*pgt = SLIST_NEXT(*pgt, link);
				/* We should have allocated enough */
				assert(*pgt);
			}
			assert((*pgt)->vabase == pg_info->va_base);
			pg_info->table = (*pgt)->tbl;

			core_mmu_set_entry(dir_info, idx,
					   virt_to_phys(pg_info->table),
					   pgt_attr);
		}

		r.size = MIN(CORE_MMU_PGDIR_SIZE - (r.va - pg_info->va_base),
			     end - r.va);

		if (!(*pgt)->populated  && !mobj_is_paged(region->mobj)) {
			size_t granule = BIT(pg_info->shift);
			size_t offset = r.va - region->va + region->offset;

			r.size = MIN(r.size,
				     mobj_get_phys_granule(region->mobj));
			r.size = ROUNDUP(r.size, SMALL_PAGE_SIZE);

			if (mobj_get_pa(region->mobj, offset, granule,
					&r.pa) != TEE_SUCCESS)
				panic("Failed to get PA of unpaged mobj");
			set_region(pg_info, &r);
		}
		r.va += r.size;
	}
}

static bool can_map_at_level(paddr_t paddr, vaddr_t vaddr,
			     size_t size_left, paddr_t block_size,
			     struct tee_mmap_region *mm)
{
	/* VA and PA are aligned to block size at current level */
	if ((vaddr | paddr) & (block_size - 1))
		return false;

	/* Remainder fits into block at current level */
	if (size_left < block_size)
		return false;

	/*
	 * The required block size of the region is compatible with the
	 * block size of the current level.
	 */
	if (mm->region_size < block_size)
		return false;

#ifdef CFG_WITH_PAGER
	/*
	 * If pager is enabled, we need to map TEE RAM and the whole pager
	 * regions with small pages only
	 */
	if ((map_is_tee_ram(mm) || mm->type == MEM_AREA_PAGER_VASPACE) &&
	    block_size != SMALL_PAGE_SIZE)
		return false;
#endif

	return true;
}

void core_mmu_map_region(struct mmu_partition *prtn, struct tee_mmap_region *mm)
{
	struct core_mmu_table_info tbl_info = { };
	unsigned int idx = 0;
	vaddr_t vaddr = mm->va;
	paddr_t paddr = mm->pa;
	ssize_t size_left = mm->size;
	uint32_t attr = mm->attr;
	unsigned int level = 0;
	bool table_found = false;
	uint32_t old_attr = 0;

	assert(!((vaddr | paddr) & SMALL_PAGE_MASK));
	if (!paddr)
		attr = 0;

	while (size_left > 0) {
		level = CORE_MMU_BASE_TABLE_LEVEL;

		while (true) {
			paddr_t block_size = 0;

			assert(core_mmu_level_in_range(level));

			table_found = core_mmu_find_table(prtn, vaddr, level,
							  &tbl_info);
			if (!table_found)
				panic("can't find table for mapping");

			block_size = BIT64(tbl_info.shift);

			idx = core_mmu_va2idx(&tbl_info, vaddr);
			if (!can_map_at_level(paddr, vaddr, size_left,
					      block_size, mm)) {
				bool secure = mm->attr & TEE_MATTR_SECURE;

				/*
				 * This part of the region can't be mapped at
				 * this level. Need to go deeper.
				 */
				if (!core_mmu_entry_to_finer_grained(&tbl_info,
								     idx,
								     secure))
					panic("Can't divide MMU entry");
				level = tbl_info.next_level;
				continue;
			}

			/* We can map part of the region at current level */
			core_mmu_get_entry(&tbl_info, idx, NULL, &old_attr);
			if (old_attr)
				panic("Page is already mapped");

			core_mmu_set_entry(&tbl_info, idx, paddr, attr);
			/*
			 * Dynamic vaspace regions don't have a physical
			 * address initially but we need to allocate and
			 * initialize the translation tables now for later
			 * updates to work properly.
			 */
			if (paddr)
				paddr += block_size;
			vaddr += block_size;
			size_left -= block_size;

			break;
		}
	}
}

TEE_Result core_mmu_map_pages(vaddr_t vstart, paddr_t *pages, size_t num_pages,
			      enum teecore_memtypes memtype)
{
	TEE_Result ret;
	struct core_mmu_table_info tbl_info;
	struct tee_mmap_region *mm;
	unsigned int idx;
	uint32_t old_attr;
	uint32_t exceptions;
	vaddr_t vaddr = vstart;
	size_t i;
	bool secure;

	assert(!(core_mmu_type_to_attr(memtype) & TEE_MATTR_PX));

	secure = core_mmu_type_to_attr(memtype) & TEE_MATTR_SECURE;

	if (vaddr & SMALL_PAGE_MASK)
		return TEE_ERROR_BAD_PARAMETERS;

	exceptions = mmu_lock();

	mm = find_map_by_va((void *)vaddr);
	if (!mm || !va_is_in_map(mm, vaddr + num_pages * SMALL_PAGE_SIZE - 1))
		panic("VA does not belong to any known mm region");

	if (!core_mmu_is_dynamic_vaspace(mm))
		panic("Trying to map into static region");

	for (i = 0; i < num_pages; i++) {
		if (pages[i] & SMALL_PAGE_MASK) {
			ret = TEE_ERROR_BAD_PARAMETERS;
			goto err;
		}

		while (true) {
			if (!core_mmu_find_table(NULL, vaddr, UINT_MAX,
						 &tbl_info))
				panic("Can't find pagetable for vaddr ");

			idx = core_mmu_va2idx(&tbl_info, vaddr);
			if (tbl_info.shift == SMALL_PAGE_SHIFT)
				break;

			/* This is supertable. Need to divide it. */
			if (!core_mmu_entry_to_finer_grained(&tbl_info, idx,
							     secure))
				panic("Failed to spread pgdir on small tables");
		}

		core_mmu_get_entry(&tbl_info, idx, NULL, &old_attr);
		if (old_attr)
			panic("Page is already mapped");

		core_mmu_set_entry(&tbl_info, idx, pages[i],
				   core_mmu_type_to_attr(memtype));
		vaddr += SMALL_PAGE_SIZE;
	}

	/*
	 * Make sure all the changes to translation tables are visible
	 * before returning. TLB doesn't need to be invalidated as we are
	 * guaranteed that there's no valid mapping in this range.
	 */
	core_mmu_table_write_barrier();
	mmu_unlock(exceptions);

	return TEE_SUCCESS;
err:
	mmu_unlock(exceptions);

	if (i)
		core_mmu_unmap_pages(vstart, i);

	return ret;
}

TEE_Result core_mmu_map_contiguous_pages(vaddr_t vstart, paddr_t pstart,
					 size_t num_pages,
					 enum teecore_memtypes memtype)
{
	struct core_mmu_table_info tbl_info = { };
	struct tee_mmap_region *mm = NULL;
	unsigned int idx = 0;
	uint32_t old_attr = 0;
	uint32_t exceptions = 0;
	vaddr_t vaddr = vstart;
	paddr_t paddr = pstart;
	size_t i = 0;
	bool secure = false;

	assert(!(core_mmu_type_to_attr(memtype) & TEE_MATTR_PX));

	secure = core_mmu_type_to_attr(memtype) & TEE_MATTR_SECURE;

	if ((vaddr | paddr) & SMALL_PAGE_MASK)
		return TEE_ERROR_BAD_PARAMETERS;

	exceptions = mmu_lock();

	mm = find_map_by_va((void *)vaddr);
	if (!mm || !va_is_in_map(mm, vaddr + num_pages * SMALL_PAGE_SIZE - 1))
		panic("VA does not belong to any known mm region");

	if (!core_mmu_is_dynamic_vaspace(mm))
		panic("Trying to map into static region");

	for (i = 0; i < num_pages; i++) {
		while (true) {
			if (!core_mmu_find_table(NULL, vaddr, UINT_MAX,
						 &tbl_info))
				panic("Can't find pagetable for vaddr ");

			idx = core_mmu_va2idx(&tbl_info, vaddr);
			if (tbl_info.shift == SMALL_PAGE_SHIFT)
				break;

			/* This is supertable. Need to divide it. */
			if (!core_mmu_entry_to_finer_grained(&tbl_info, idx,
							     secure))
				panic("Failed to spread pgdir on small tables");
		}

		core_mmu_get_entry(&tbl_info, idx, NULL, &old_attr);
		if (old_attr)
			panic("Page is already mapped");

		core_mmu_set_entry(&tbl_info, idx, paddr,
				   core_mmu_type_to_attr(memtype));
		paddr += SMALL_PAGE_SIZE;
		vaddr += SMALL_PAGE_SIZE;
	}

	/*
	 * Make sure all the changes to translation tables are visible
	 * before returning. TLB doesn't need to be invalidated as we are
	 * guaranteed that there's no valid mapping in this range.
	 */
	core_mmu_table_write_barrier();
	mmu_unlock(exceptions);

	return TEE_SUCCESS;
}

static bool mem_range_is_in_vcore_free(vaddr_t vstart, size_t num_pages)
{
	return core_is_buffer_inside(vstart, num_pages * SMALL_PAGE_SIZE,
				     VCORE_FREE_PA, VCORE_FREE_SZ);
}

static void maybe_remove_from_mem_map(vaddr_t vstart, size_t num_pages)
{
	struct memory_map *mem_map = NULL;
	struct tee_mmap_region *mm = NULL;
	size_t idx = 0;
	vaddr_t va = 0;

	mm = find_map_by_va((void *)vstart);
	if (!mm || !va_is_in_map(mm, vstart + num_pages * SMALL_PAGE_SIZE - 1))
		panic("VA does not belong to any known mm region");

	if (core_mmu_is_dynamic_vaspace(mm))
		return;

	if (!mem_range_is_in_vcore_free(vstart, num_pages))
		panic("Trying to unmap static region");

	/*
	 * We're going to remove a memory from the VCORE_FREE memory range.
	 * Depending where the range is we may need to remove the matching
	 * mm, peal of a bit from the start or end of the mm, or split it
	 * into two with a whole in the middle.
	 */

	va = ROUNDDOWN(vstart, SMALL_PAGE_SIZE);
	assert(mm->region_size == SMALL_PAGE_SIZE);

	if (va == mm->va && mm->size == num_pages * SMALL_PAGE_SIZE) {
		mem_map = get_memory_map();
		idx = mm - mem_map->map;
		assert(idx < mem_map->count);

		rem_array_elem(mem_map->map, mem_map->count,
			       sizeof(*mem_map->map), idx);
		mem_map->count--;
	} else if (va == mm->va) {
		mm->va += num_pages * SMALL_PAGE_SIZE;
		mm->pa += num_pages * SMALL_PAGE_SIZE;
		mm->size -= num_pages * SMALL_PAGE_SIZE;
	} else if (va + num_pages * SMALL_PAGE_SIZE == mm->va + mm->size) {
		mm->size -= num_pages * SMALL_PAGE_SIZE;
	} else {
		struct tee_mmap_region m = *mm;

		mem_map = get_memory_map();
		idx = mm - mem_map->map;
		assert(idx < mem_map->count);

		mm->size = va - mm->va;
		m.va += mm->size + num_pages * SMALL_PAGE_SIZE;
		m.pa += mm->size + num_pages * SMALL_PAGE_SIZE;
		m.size -= mm->size + num_pages * SMALL_PAGE_SIZE;
		grow_mem_map(mem_map);
		ins_array_elem(mem_map->map, mem_map->count,
			       sizeof(*mem_map->map), idx + 1, &m);
	}
}

void core_mmu_unmap_pages(vaddr_t vstart, size_t num_pages)
{
	struct core_mmu_table_info tbl_info;
	size_t i;
	unsigned int idx;
	uint32_t exceptions;

	exceptions = mmu_lock();

	maybe_remove_from_mem_map(vstart, num_pages);

	for (i = 0; i < num_pages; i++, vstart += SMALL_PAGE_SIZE) {
		if (!core_mmu_find_table(NULL, vstart, UINT_MAX, &tbl_info))
			panic("Can't find pagetable");

		if (tbl_info.shift != SMALL_PAGE_SHIFT)
			panic("Invalid pagetable level");

		idx = core_mmu_va2idx(&tbl_info, vstart);
		core_mmu_set_entry(&tbl_info, idx, 0, 0);
	}
	tlbi_all();

	mmu_unlock(exceptions);
}

void core_mmu_populate_user_map(struct core_mmu_table_info *dir_info,
				struct user_mode_ctx *uctx)
{
	struct core_mmu_table_info pg_info = { };
	struct pgt_cache *pgt_cache = &uctx->pgt_cache;
	struct pgt *pgt = NULL;
	struct pgt *p = NULL;
	struct vm_region *r = NULL;

	if (TAILQ_EMPTY(&uctx->vm_info.regions))
		return; /* Nothing to map */

	/*
	 * Allocate all page tables in advance.
	 */
	pgt_get_all(uctx);
	pgt = SLIST_FIRST(pgt_cache);

	core_mmu_set_info_table(&pg_info, dir_info->next_level, 0, NULL);

	TAILQ_FOREACH(r, &uctx->vm_info.regions, link)
		set_pg_region(dir_info, r, &pgt, &pg_info);
	/* Record that the translation tables now are populated. */
	SLIST_FOREACH(p, pgt_cache, link) {
		p->populated = true;
		if (p == pgt)
			break;
	}
	assert(p == pgt);
}

TEE_Result core_mmu_remove_mapping(enum teecore_memtypes type, void *addr,
				   size_t len)
{
	struct core_mmu_table_info tbl_info = { };
	struct tee_mmap_region *res_map = NULL;
	struct tee_mmap_region *map = NULL;
	paddr_t pa = virt_to_phys(addr);
	size_t granule = 0;
	ptrdiff_t i = 0;
	paddr_t p = 0;
	size_t l = 0;

	map = find_map_by_type_and_pa(type, pa, len);
	if (!map)
		return TEE_ERROR_GENERIC;

	res_map = find_map_by_type(MEM_AREA_RES_VASPACE);
	if (!res_map)
		return TEE_ERROR_GENERIC;
	if (!core_mmu_find_table(NULL, res_map->va, UINT_MAX, &tbl_info))
		return TEE_ERROR_GENERIC;
	granule = BIT(tbl_info.shift);

	if (map < static_memory_map.map ||
	    map >= static_memory_map.map + static_memory_map.count)
		return TEE_ERROR_GENERIC;
	i = map - static_memory_map.map;

	/* Check that we have a full match */
	p = ROUNDDOWN2(pa, granule);
	l = ROUNDUP2(len + pa - p, granule);
	if (map->pa != p || map->size != l)
		return TEE_ERROR_GENERIC;

	clear_region(&tbl_info, map);
	tlbi_all();

	/* If possible remove the va range from res_map */
	if (res_map->va - map->size == map->va) {
		res_map->va -= map->size;
		res_map->size += map->size;
	}

	/* Remove the entry. */
	rem_array_elem(static_memory_map.map, static_memory_map.count,
		       sizeof(*static_memory_map.map), i);
	static_memory_map.count--;

	return TEE_SUCCESS;
}

struct tee_mmap_region *
core_mmu_find_mapping_exclusive(enum teecore_memtypes type, size_t len)
{
	struct memory_map *mem_map = get_memory_map();
	struct tee_mmap_region *map_found = NULL;
	size_t n = 0;

	if (!len)
		return NULL;

	for (n = 0; n < mem_map->count; n++) {
		if (mem_map->map[n].type != type)
			continue;

		if (map_found)
			return NULL;

		map_found = mem_map->map + n;
	}

	if (!map_found || map_found->size < len)
		return NULL;

	return map_found;
}

void *core_mmu_add_mapping(enum teecore_memtypes type, paddr_t addr, size_t len)
{
	struct memory_map *mem_map = &static_memory_map;
	struct core_mmu_table_info tbl_info = { };
	struct tee_mmap_region *map = NULL;
	size_t granule = 0;
	paddr_t p = 0;
	size_t l = 0;

	if (!len)
		return NULL;

	if (!core_mmu_check_end_pa(addr, len))
		return NULL;

	/* Check if the memory is already mapped */
	map = find_map_by_type_and_pa(type, addr, len);
	if (map && pbuf_inside_map_area(addr, len, map))
		return (void *)(vaddr_t)(map->va + addr - map->pa);

	/* Find the reserved va space used for late mappings */
	map = find_map_by_type(MEM_AREA_RES_VASPACE);
	if (!map)
		return NULL;

	if (!core_mmu_find_table(NULL, map->va, UINT_MAX, &tbl_info))
		return NULL;

	granule = BIT64(tbl_info.shift);
	p = ROUNDDOWN2(addr, granule);
	l = ROUNDUP2(len + addr - p, granule);

	/* Ban overflowing virtual addresses */
	if (map->size < l)
		return NULL;

	/*
	 * Something is wrong, we can't fit the va range into the selected
	 * table. The reserved va range is possibly missaligned with
	 * granule.
	 */
	if (core_mmu_va2idx(&tbl_info, map->va + len) >= tbl_info.num_entries)
		return NULL;

	if (static_memory_map.count >= static_memory_map.alloc_count)
		return NULL;

	mem_map->map[mem_map->count] = (struct tee_mmap_region){
		.va = map->va,
		.size = l,
		.type = type,
		.region_size = granule,
		.attr = core_mmu_type_to_attr(type),
		.pa = p,
	};
	map->va += l;
	map->size -= l;
	map = mem_map->map + mem_map->count;
	mem_map->count++;

	set_region(&tbl_info, map);

	/* Make sure the new entry is visible before continuing. */
	core_mmu_table_write_barrier();

	return (void *)(vaddr_t)(map->va + addr - map->pa);
}

#ifdef CFG_WITH_PAGER
static vaddr_t get_linear_map_end_va(void)
{
	/* this is synced with the generic linker file kern.ld.S */
	return (vaddr_t)__heap2_end;
}

static paddr_t get_linear_map_end_pa(void)
{
	return get_linear_map_end_va() - boot_mmu_config.map_offset;
}
#endif

#if defined(CFG_TEE_CORE_DEBUG)
static void check_pa_matches_va(void *va, paddr_t pa)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	vaddr_t v = (vaddr_t)va;
	paddr_t p = 0;
	struct core_mmu_table_info ti __maybe_unused = { };

	if (core_mmu_user_va_range_is_defined()) {
		vaddr_t user_va_base = 0;
		size_t user_va_size = 0;

		core_mmu_get_user_va_range(&user_va_base, &user_va_size);
		if (v >= user_va_base &&
		    v <= (user_va_base - 1 + user_va_size)) {
			if (!core_mmu_user_mapping_is_active()) {
				if (pa)
					panic("issue in linear address space");
				return;
			}

			res = vm_va2pa(to_user_mode_ctx(thread_get_tsd()->ctx),
				       va, &p);
			if (res == TEE_ERROR_NOT_SUPPORTED)
				return;
			if (res == TEE_SUCCESS && pa != p)
				panic("bad pa");
			if (res != TEE_SUCCESS && pa)
				panic("false pa");
			return;
		}
	}
#ifdef CFG_WITH_PAGER
	if (is_unpaged(va)) {
		if (v - boot_mmu_config.map_offset != pa)
			panic("issue in linear address space");
		return;
	}

	if (tee_pager_get_table_info(v, &ti)) {
		uint32_t a;

		/*
		 * Lookups in the page table managed by the pager is
		 * dangerous for addresses in the paged area as those pages
		 * changes all the time. But some ranges are safe,
		 * rw-locked areas when the page is populated for instance.
		 */
		core_mmu_get_entry(&ti, core_mmu_va2idx(&ti, v), &p, &a);
		if (a & TEE_MATTR_VALID_BLOCK) {
			paddr_t mask = BIT64(ti.shift) - 1;

			p |= v & mask;
			if (pa != p)
				panic();
		} else {
			if (pa)
				panic();
		}
		return;
	}
#endif

	if (!core_va2pa_helper(va, &p)) {
		/* Verfiy only the static mapping (case non null phys addr) */
		if (p && pa != p) {
			DMSG("va %p maps 0x%" PRIxPA ", expect 0x%" PRIxPA,
			     va, p, pa);
			panic();
		}
	} else {
		if (pa) {
			DMSG("va %p unmapped, expect 0x%" PRIxPA, va, pa);
			panic();
		}
	}
}
#else
static void check_pa_matches_va(void *va __unused, paddr_t pa __unused)
{
}
#endif

paddr_t virt_to_phys(void *va)
{
	paddr_t pa = 0;

	if (!arch_va2pa_helper(va, &pa))
		pa = 0;
	check_pa_matches_va(memtag_strip_tag(va), pa);
	return pa;
}

/*
 * Don't use check_va_matches_pa() for RISC-V, as its callee
 * arch_va2pa_helper() will call it eventually, this creates
 * indirect recursion and can lead to a stack overflow.
 * Moreover, if arch_va2pa_helper() returns true, it implies
 * the va2pa mapping is matched, no need to check it again.
 */
#if defined(CFG_TEE_CORE_DEBUG) && !defined(__riscv)
static void check_va_matches_pa(paddr_t pa, void *va)
{
	paddr_t p = 0;

	if (!va)
		return;

	p = virt_to_phys(va);
	if (p != pa) {
		DMSG("va %p maps 0x%" PRIxPA " expect 0x%" PRIxPA, va, p, pa);
		panic();
	}
}
#else
static void check_va_matches_pa(paddr_t pa __unused, void *va __unused)
{
}
#endif

static void *phys_to_virt_ts_vaspace(paddr_t pa, size_t len)
{
	if (!core_mmu_user_mapping_is_active())
		return NULL;

	return vm_pa2va(to_user_mode_ctx(thread_get_tsd()->ctx), pa, len);
}

#ifdef CFG_WITH_PAGER
static void *phys_to_virt_tee_ram(paddr_t pa, size_t len)
{
	paddr_t end_pa = 0;

	if (SUB_OVERFLOW(len, 1, &end_pa) || ADD_OVERFLOW(pa, end_pa, &end_pa))
		return NULL;

	if (pa >= TEE_LOAD_ADDR && pa < get_linear_map_end_pa()) {
		if (end_pa > get_linear_map_end_pa())
			return NULL;
		return (void *)(vaddr_t)(pa + boot_mmu_config.map_offset);
	}

	return tee_pager_phys_to_virt(pa, len);
}
#else
static void *phys_to_virt_tee_ram(paddr_t pa, size_t len)
{
	struct tee_mmap_region *mmap = NULL;

	mmap = find_map_by_type_and_pa(MEM_AREA_TEE_RAM, pa, len);
	if (!mmap)
		mmap = find_map_by_type_and_pa(MEM_AREA_NEX_RAM_RW, pa, len);
	if (!mmap)
		mmap = find_map_by_type_and_pa(MEM_AREA_NEX_RAM_RO, pa, len);
	if (!mmap)
		mmap = find_map_by_type_and_pa(MEM_AREA_TEE_RAM_RW, pa, len);
	if (!mmap)
		mmap = find_map_by_type_and_pa(MEM_AREA_TEE_RAM_RO, pa, len);
	if (!mmap)
		mmap = find_map_by_type_and_pa(MEM_AREA_TEE_RAM_RX, pa, len);

	/*
	 * Note that MEM_AREA_INIT_RAM_RO and MEM_AREA_INIT_RAM_RX are only
	 * used with pager and not needed here.
	 */
	return map_pa2va(mmap, pa, len);
}
#endif

void *phys_to_virt(paddr_t pa, enum teecore_memtypes m, size_t len)
{
	void *va = NULL;

	switch (m) {
	case MEM_AREA_TS_VASPACE:
		va = phys_to_virt_ts_vaspace(pa, len);
		break;
	case MEM_AREA_TEE_RAM:
	case MEM_AREA_TEE_RAM_RX:
	case MEM_AREA_TEE_RAM_RO:
	case MEM_AREA_TEE_RAM_RW:
	case MEM_AREA_NEX_RAM_RO:
	case MEM_AREA_NEX_RAM_RW:
		va = phys_to_virt_tee_ram(pa, len);
		break;
	case MEM_AREA_SHM_VASPACE:
	case MEM_AREA_NEX_DYN_VASPACE:
	case MEM_AREA_TEE_DYN_VASPACE:
		/* Find VA from PA in dynamic SHM is not yet supported */
		va = NULL;
		break;
	default:
		va = map_pa2va(find_map_by_type_and_pa(m, pa, len), pa, len);
	}
	if (m != MEM_AREA_SEC_RAM_OVERALL)
		check_va_matches_pa(pa, va);
	return va;
}

void *phys_to_virt_io(paddr_t pa, size_t len)
{
	struct tee_mmap_region *map = NULL;
	void *va = NULL;

	map = find_map_by_type_and_pa(MEM_AREA_IO_SEC, pa, len);
	if (!map)
		map = find_map_by_type_and_pa(MEM_AREA_IO_NSEC, pa, len);
	if (!map)
		return NULL;
	va = map_pa2va(map, pa, len);
	check_va_matches_pa(pa, va);
	return va;
}

vaddr_t core_mmu_get_va(paddr_t pa, enum teecore_memtypes type, size_t len)
{
	if (cpu_mmu_enabled())
		return (vaddr_t)phys_to_virt(pa, type, len);

	return (vaddr_t)pa;
}

#ifdef CFG_WITH_PAGER
bool is_unpaged(const void *va)
{
	vaddr_t v = (vaddr_t)va;

	return v >= VCORE_START_VA && v < get_linear_map_end_va();
}
#endif

#ifdef CFG_NS_VIRTUALIZATION
bool is_nexus(const void *va)
{
	vaddr_t v = (vaddr_t)va;

	return v >= VCORE_START_VA && v < VCORE_NEX_RW_PA + VCORE_NEX_RW_SZ;
}
#endif

vaddr_t io_pa_or_va(struct io_pa_va *p, size_t len)
{
	assert(p->pa);
	if (cpu_mmu_enabled()) {
		if (!p->va)
			p->va = (vaddr_t)phys_to_virt_io(p->pa, len);
		assert(p->va);
		return p->va;
	}
	return p->pa;
}

vaddr_t io_pa_or_va_secure(struct io_pa_va *p, size_t len)
{
	assert(p->pa);
	if (cpu_mmu_enabled()) {
		if (!p->va)
			p->va = (vaddr_t)phys_to_virt(p->pa, MEM_AREA_IO_SEC,
						      len);
		assert(p->va);
		return p->va;
	}
	return p->pa;
}

vaddr_t io_pa_or_va_nsec(struct io_pa_va *p, size_t len)
{
	assert(p->pa);
	if (cpu_mmu_enabled()) {
		if (!p->va)
			p->va = (vaddr_t)phys_to_virt(p->pa, MEM_AREA_IO_NSEC,
						      len);
		assert(p->va);
		return p->va;
	}
	return p->pa;
}

#ifdef CFG_CORE_RESERVED_SHM
static TEE_Result teecore_init_pub_ram(void)
{
	vaddr_t s = 0;
	vaddr_t e = 0;

	/* get virtual addr/size of NSec shared mem allocated from teecore */
	core_mmu_get_mem_by_type(MEM_AREA_NSEC_SHM, &s, &e);

	if (s >= e || s & SMALL_PAGE_MASK || e & SMALL_PAGE_MASK)
		panic("invalid PUB RAM");

	/* extra check: we could rely on core_mmu_get_mem_by_type() */
	if (!tee_vbuf_is_non_sec(s, e - s))
		panic("PUB RAM is not non-secure");

#ifdef CFG_PL310
	/* Allocate statically the l2cc mutex */
	tee_l2cc_store_mutex_boot_pa(virt_to_phys((void *)s));
	s += sizeof(uint32_t);			/* size of a pl310 mutex */
	s = ROUNDUP(s, SMALL_PAGE_SIZE);	/* keep required alignment */
#endif

	default_nsec_shm_paddr = virt_to_phys((void *)s);
	default_nsec_shm_size = e - s;

	return TEE_SUCCESS;
}
early_init(teecore_init_pub_ram);
#endif /*CFG_CORE_RESERVED_SHM*/

static void __maybe_unused carve_out_core_mem(paddr_t pa, paddr_t end_pa)
{
	tee_mm_entry_t *mm __maybe_unused = NULL;

	DMSG("%#"PRIxPA" .. %#"PRIxPA, pa, end_pa);
	mm = phys_mem_alloc2(pa, end_pa - pa);
	assert(mm);
}

void core_mmu_init_phys_mem(void)
{
	if (IS_ENABLED(CFG_NS_VIRTUALIZATION)) {
		paddr_t b1 = 0;
		paddr_size_t s1 = 0;

		static_assert(ARRAY_SIZE(secure_only) <= 2);

		if (ARRAY_SIZE(secure_only) == 2) {
			b1 = secure_only[1].paddr;
			s1 = secure_only[1].size;
		}
		virt_init_memory(&static_memory_map, secure_only[0].paddr,
				 secure_only[0].size, b1, s1);
	} else {
#ifdef CFG_WITH_PAGER
		/*
		 * The pager uses all core memory so there's no need to add
		 * it to the pool.
		 */
		static_assert(ARRAY_SIZE(secure_only) == 2);
		phys_mem_init(0, 0, secure_only[1].paddr, secure_only[1].size);
#else /*!CFG_WITH_PAGER*/
		size_t align = BIT(CORE_MMU_USER_CODE_SHIFT);
		paddr_t end_pa = 0;
		size_t size = 0;
		paddr_t ps = 0;
		paddr_t pa = 0;

		static_assert(ARRAY_SIZE(secure_only) <= 2);
		if (ARRAY_SIZE(secure_only) == 2) {
			ps = secure_only[1].paddr;
			size = secure_only[1].size;
		}
		phys_mem_init(secure_only[0].paddr, secure_only[0].size,
			      ps, size);

		/*
		 * The VCORE macros are relocatable so we need to translate
		 * the addresses now that the MMU is enabled.
		 */
		end_pa = vaddr_to_phys(ROUNDUP2(VCORE_FREE_END_PA,
						align) - 1) + 1;
		/* Carve out the part used by OP-TEE core */
		carve_out_core_mem(vaddr_to_phys(VCORE_UNPG_RX_PA), end_pa);
		if (IS_ENABLED(CFG_CORE_SANITIZE_KADDRESS)) {
			pa = vaddr_to_phys(ROUNDUP2(ASAN_MAP_PA, align));
			carve_out_core_mem(pa, pa + ASAN_MAP_SZ);
		}

		/* Carve out test SDP memory */
#ifdef TEE_SDP_TEST_MEM_BASE
		if (TEE_SDP_TEST_MEM_SIZE) {
			pa = TEE_SDP_TEST_MEM_BASE;
			carve_out_core_mem(pa, pa + TEE_SDP_TEST_MEM_SIZE);
		}
#endif
#endif /*!CFG_WITH_PAGER*/
	}
}
