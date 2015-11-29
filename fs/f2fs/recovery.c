/*
 * fs/f2fs/recovery.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/fs.h>
#include <linux/f2fs_fs.h>
#include "f2fs.h"
#include "node.h"
#include "segment.h"

/*
 * Roll forward recovery scenarios.
 *
 * [Term] F: fsync_mark, D: dentry_mark
 *
 * 1. inode(x) | CP | inode(x) | dnode(F)
 * -> Update the latest inode(x).
 *
 * 2. inode(x) | CP | inode(F) | dnode(F)
 * -> No problem.
 *
 * 3. inode(x) | CP | dnode(F) | inode(x)
 * -> Recover to the latest dnode(F), and drop the last inode(x)
 *
 * 4. inode(x) | CP | dnode(F) | inode(F)
 * -> No problem.
 *
 * 5. CP | inode(x) | dnode(F)
 * -> The inode(DF) was missing. Should drop this dnode(F).
 *
 * 6. CP | inode(DF) | dnode(F)
 * -> No problem.
 *
 * 7. CP | dnode(F) | inode(DF)
 * -> If f2fs_iget fails, then goto next to find inode(DF).
 *
 * 8. CP | dnode(F) | inode(x)
 * -> If f2fs_iget fails, then goto next to find inode(DF).
 *    But it will fail due to no inode(DF).
 */

static struct kmem_cache *fsync_entry_slab;

bool space_for_roll_forward(struct f2fs_sb_info *sbi)
{
<<<<<<< HEAD
	if (sbi->last_valid_block_count + sbi->alloc_valid_block_count
			> sbi->user_block_count)
=======
	s64 nalloc = percpu_counter_sum_positive(&sbi->alloc_valid_block_count);

	if (sbi->last_valid_block_count + nalloc > sbi->user_block_count)
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
		return false;
	return true;
}

static struct fsync_inode_entry *get_fsync_inode(struct list_head *head,
								nid_t ino)
{
	struct fsync_inode_entry *entry;

	list_for_each_entry(entry, head, list)
		if (entry->inode->i_ino == ino)
			return entry;

	return NULL;
}

<<<<<<< HEAD
static int recover_dentry(struct inode *inode, struct page *ipage)
=======
static struct fsync_inode_entry *add_fsync_inode(struct list_head *head,
							struct inode *inode)
{
	struct fsync_inode_entry *entry;

	entry = kmem_cache_alloc(fsync_entry_slab, GFP_F2FS_ZERO);
	if (!entry)
		return NULL;

	entry->inode = inode;
	list_add_tail(&entry->list, head);

	return entry;
}

static void del_fsync_inode(struct fsync_inode_entry *entry)
{
	iput(entry->inode);
	list_del(&entry->list);
	kmem_cache_free(fsync_entry_slab, entry);
}

static int recover_dentry(struct inode *inode, struct page *ipage,
						struct list_head *dir_list)
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
{
	struct f2fs_inode *raw_inode = F2FS_INODE(ipage);
	nid_t pino = le32_to_cpu(raw_inode->i_pino);
	struct f2fs_dir_entry *de;
	struct qstr name;
	struct page *page;
	struct inode *dir, *einode;
<<<<<<< HEAD
	int err = 0;

	dir = f2fs_iget(inode->i_sb, pino);
	if (IS_ERR(dir)) {
		err = PTR_ERR(dir);
		goto out;
	}

=======
	struct fsync_inode_entry *entry;
	int err = 0;

	entry = get_fsync_inode(dir_list, pino);
	if (!entry) {
		dir = f2fs_iget(inode->i_sb, pino);
		if (IS_ERR(dir)) {
			err = PTR_ERR(dir);
			goto out;
		}

		entry = add_fsync_inode(dir_list, dir);
		if (!entry) {
			err = -ENOMEM;
			iput(dir);
			goto out;
		}
	}

	dir = entry->inode;

	if (file_enc_name(inode))
		return 0;

>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	name.len = le32_to_cpu(raw_inode->i_namelen);
	name.name = raw_inode->i_name;

	if (unlikely(name.len > F2FS_NAME_LEN)) {
		WARN_ON(1);
		err = -ENAMETOOLONG;
<<<<<<< HEAD
		goto out_err;
=======
		goto out;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	}
retry:
	de = f2fs_find_entry(dir, &name, &page);
	if (de && inode->i_ino == le32_to_cpu(de->ino))
		goto out_unmap_put;

	if (de) {
		einode = f2fs_iget(inode->i_sb, le32_to_cpu(de->ino));
		if (IS_ERR(einode)) {
			WARN_ON(1);
			err = PTR_ERR(einode);
			if (err == -ENOENT)
				err = -EEXIST;
			goto out_unmap_put;
		}
		err = acquire_orphan_inode(F2FS_I_SB(inode));
		if (err) {
			iput(einode);
			goto out_unmap_put;
		}
		f2fs_delete_entry(de, page, dir, einode);
		iput(einode);
		goto retry;
<<<<<<< HEAD
	}
	err = __f2fs_add_link(dir, &name, inode, inode->i_ino, inode->i_mode);
	if (err)
		goto out_err;

	if (is_inode_flag_set(F2FS_I(dir), FI_DELAY_IPUT)) {
		iput(dir);
	} else {
		add_dirty_dir_inode(dir);
		set_inode_flag(F2FS_I(dir), FI_DELAY_IPUT);
	}

=======
	} else if (IS_ERR(page)) {
		err = PTR_ERR(page);
	} else {
		err = __f2fs_add_link(dir, &name, inode,
					inode->i_ino, inode->i_mode);
	}
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	goto out;

out_unmap_put:
	f2fs_dentry_kunmap(dir, page);
	f2fs_put_page(page, 0);
<<<<<<< HEAD
out_err:
	iput(dir);
=======
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
out:
	f2fs_msg(inode->i_sb, KERN_NOTICE,
			"%s: ino = %x, name = %s, dir = %lx, err = %d",
			__func__, ino_of_node(ipage), raw_inode->i_name,
			IS_ERR(dir) ? 0 : dir->i_ino, err);
	return err;
}

static void recover_inode(struct inode *inode, struct page *page)
{
	struct f2fs_inode *raw = F2FS_INODE(page);
<<<<<<< HEAD

	inode->i_mode = le16_to_cpu(raw->i_mode);
	i_size_write(inode, le64_to_cpu(raw->i_size));
=======
	char *name;

	inode->i_mode = le16_to_cpu(raw->i_mode);
	f2fs_i_size_write(inode, le64_to_cpu(raw->i_size));
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	inode->i_atime.tv_sec = le64_to_cpu(raw->i_mtime);
	inode->i_ctime.tv_sec = le64_to_cpu(raw->i_ctime);
	inode->i_mtime.tv_sec = le64_to_cpu(raw->i_mtime);
	inode->i_atime.tv_nsec = le32_to_cpu(raw->i_mtime_nsec);
	inode->i_ctime.tv_nsec = le32_to_cpu(raw->i_ctime_nsec);
	inode->i_mtime.tv_nsec = le32_to_cpu(raw->i_mtime_nsec);

<<<<<<< HEAD
	f2fs_msg(inode->i_sb, KERN_NOTICE, "recover_inode: ino = %x, name = %s",
			ino_of_node(page), F2FS_INODE(page)->i_name);
=======
	if (file_enc_name(inode))
		name = "<encrypted>";
	else
		name = F2FS_INODE(page)->i_name;

	f2fs_msg(inode->i_sb, KERN_NOTICE, "recover_inode: ino = %x, name = %s",
			ino_of_node(page), name);
}

static bool is_same_inode(struct inode *inode, struct page *ipage)
{
	struct f2fs_inode *ri = F2FS_INODE(ipage);
	struct timespec disk;

	if (!IS_INODE(ipage))
		return true;

	disk.tv_sec = le64_to_cpu(ri->i_ctime);
	disk.tv_nsec = le32_to_cpu(ri->i_ctime_nsec);
	if (timespec_compare(&inode->i_ctime, &disk) > 0)
		return false;

	disk.tv_sec = le64_to_cpu(ri->i_atime);
	disk.tv_nsec = le32_to_cpu(ri->i_atime_nsec);
	if (timespec_compare(&inode->i_atime, &disk) > 0)
		return false;

	disk.tv_sec = le64_to_cpu(ri->i_mtime);
	disk.tv_nsec = le32_to_cpu(ri->i_mtime_nsec);
	if (timespec_compare(&inode->i_mtime, &disk) > 0)
		return false;

	return true;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
}

static int find_fsync_dnodes(struct f2fs_sb_info *sbi, struct list_head *head)
{
	unsigned long long cp_ver = cur_cp_version(F2FS_CKPT(sbi));
	struct curseg_info *curseg;
<<<<<<< HEAD
=======
	struct inode *inode;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	struct page *page = NULL;
	block_t blkaddr;
	int err = 0;

	/* get node pages in the current segment */
	curseg = CURSEG_I(sbi, CURSEG_WARM_NODE);
	blkaddr = NEXT_FREE_BLKADDR(sbi, curseg);

<<<<<<< HEAD
	ra_meta_pages(sbi, blkaddr, 1, META_POR);

	while (1) {
		struct fsync_inode_entry *entry;

		if (blkaddr < MAIN_BLKADDR(sbi) || blkaddr >= MAX_BLKADDR(sbi))
			return 0;

		page = get_meta_page(sbi, blkaddr);
=======
	while (1) {
		struct fsync_inode_entry *entry;

		if (!is_valid_blkaddr(sbi, blkaddr, META_POR))
			return 0;

		page = get_tmp_page(sbi, blkaddr);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

		if (cp_ver != cpver_of_node(page))
			break;

		if (!is_fsync_dnode(page))
			goto next;

		entry = get_fsync_inode(head, ino_of_node(page));
<<<<<<< HEAD
		if (!entry) {
=======
		if (entry) {
			if (!is_same_inode(entry->inode, page))
				goto next;
		} else {
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
			if (IS_INODE(page) && is_dent_dnode(page)) {
				err = recover_inode_page(sbi, page);
				if (err)
					break;
			}

<<<<<<< HEAD
			/* add this fsync inode to the list */
			entry = kmem_cache_alloc(fsync_entry_slab, GFP_F2FS_ZERO);
			if (!entry) {
				err = -ENOMEM;
				break;
			}
=======
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
			/*
			 * CP | dnode(F) | inode(DF)
			 * For this case, we should not give up now.
			 */
<<<<<<< HEAD
			entry->inode = f2fs_iget(sbi->sb, ino_of_node(page));
			if (IS_ERR(entry->inode)) {
				err = PTR_ERR(entry->inode);
				kmem_cache_free(fsync_entry_slab, entry);
=======
			inode = f2fs_iget(sbi->sb, ino_of_node(page));
			if (IS_ERR(inode)) {
				err = PTR_ERR(inode);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
				if (err == -ENOENT) {
					err = 0;
					goto next;
				}
				break;
			}
<<<<<<< HEAD
			list_add_tail(&entry->list, head);
		}
		entry->blkaddr = blkaddr;

		if (IS_INODE(page)) {
			entry->last_inode = blkaddr;
			if (is_dent_dnode(page))
				entry->last_dentry = blkaddr;
		}
=======

			/* add this fsync inode to the list */
			entry = add_fsync_inode(head, inode);
			if (!entry) {
				err = -ENOMEM;
				iput(inode);
				break;
			}
		}
		entry->blkaddr = blkaddr;

		if (IS_INODE(page) && is_dent_dnode(page))
			entry->last_dentry = blkaddr;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
next:
		/* check next segment */
		blkaddr = next_blkaddr_of_node(page);
		f2fs_put_page(page, 1);

		ra_meta_pages_cond(sbi, blkaddr);
	}
	f2fs_put_page(page, 1);
	return err;
}

static void destroy_fsync_dnodes(struct list_head *head)
{
	struct fsync_inode_entry *entry, *tmp;

<<<<<<< HEAD
	list_for_each_entry_safe(entry, tmp, head, list) {
		iput(entry->inode);
		list_del(&entry->list);
		kmem_cache_free(fsync_entry_slab, entry);
	}
=======
	list_for_each_entry_safe(entry, tmp, head, list)
		del_fsync_inode(entry);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
}

static int check_index_in_prev_nodes(struct f2fs_sb_info *sbi,
			block_t blkaddr, struct dnode_of_data *dn)
{
	struct seg_entry *sentry;
	unsigned int segno = GET_SEGNO(sbi, blkaddr);
	unsigned short blkoff = GET_BLKOFF_FROM_SEG0(sbi, blkaddr);
	struct f2fs_summary_block *sum_node;
	struct f2fs_summary sum;
	struct page *sum_page, *node_page;
	struct dnode_of_data tdn = *dn;
	nid_t ino, nid;
	struct inode *inode;
	unsigned int offset;
	block_t bidx;
	int i;

	sentry = get_seg_entry(sbi, segno);
	if (!f2fs_test_bit(blkoff, sentry->cur_valid_map))
		return 0;

	/* Get the previous summary */
	for (i = CURSEG_WARM_DATA; i <= CURSEG_COLD_DATA; i++) {
		struct curseg_info *curseg = CURSEG_I(sbi, i);
		if (curseg->segno == segno) {
			sum = curseg->sum_blk->entries[blkoff];
			goto got_it;
		}
	}

	sum_page = get_sum_page(sbi, segno);
	sum_node = (struct f2fs_summary_block *)page_address(sum_page);
	sum = sum_node->entries[blkoff];
	f2fs_put_page(sum_page, 1);
got_it:
	/* Use the locked dnode page and inode */
	nid = le32_to_cpu(sum.nid);
	if (dn->inode->i_ino == nid) {
		tdn.nid = nid;
		if (!dn->inode_page_locked)
			lock_page(dn->inode_page);
		tdn.node_page = dn->inode_page;
		tdn.ofs_in_node = le16_to_cpu(sum.ofs_in_node);
		goto truncate_out;
	} else if (dn->nid == nid) {
		tdn.ofs_in_node = le16_to_cpu(sum.ofs_in_node);
		goto truncate_out;
	}

	/* Get the node page */
	node_page = get_node_page(sbi, nid);
	if (IS_ERR(node_page))
		return PTR_ERR(node_page);

	offset = ofs_of_node(node_page);
	ino = ino_of_node(node_page);
	f2fs_put_page(node_page, 1);

	if (ino != dn->inode->i_ino) {
		/* Deallocate previous index in the node page */
		inode = f2fs_iget(sbi->sb, ino);
		if (IS_ERR(inode))
			return PTR_ERR(inode);
	} else {
		inode = dn->inode;
	}

<<<<<<< HEAD
	bidx = start_bidx_of_node(offset, F2FS_I(inode)) +
			le16_to_cpu(sum.ofs_in_node);
=======
	bidx = start_bidx_of_node(offset, inode) + le16_to_cpu(sum.ofs_in_node);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

	/*
	 * if inode page is locked, unlock temporarily, but its reference
	 * count keeps alive.
	 */
	if (ino == dn->inode->i_ino && dn->inode_page_locked)
		unlock_page(dn->inode_page);

	set_new_dnode(&tdn, inode, NULL, NULL, 0);
	if (get_dnode_of_data(&tdn, bidx, LOOKUP_NODE))
		goto out;

	if (tdn.data_blkaddr == blkaddr)
		truncate_data_blocks_range(&tdn, 1);

	f2fs_put_dnode(&tdn);
out:
	if (ino != dn->inode->i_ino)
		iput(inode);
	else if (dn->inode_page_locked)
		lock_page(dn->inode_page);
	return 0;

truncate_out:
	if (datablock_addr(tdn.node_page, tdn.ofs_in_node) == blkaddr)
		truncate_data_blocks_range(&tdn, 1);
	if (dn->inode->i_ino == nid && !dn->inode_page_locked)
		unlock_page(dn->inode_page);
	return 0;
}

static int do_recover_data(struct f2fs_sb_info *sbi, struct inode *inode,
					struct page *page, block_t blkaddr)
{
<<<<<<< HEAD
	struct f2fs_inode_info *fi = F2FS_I(inode);
	unsigned int start, end;
	struct dnode_of_data dn;
	struct f2fs_summary sum;
	struct node_info ni;
=======
	struct dnode_of_data dn;
	struct node_info ni;
	unsigned int start, end;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	int err = 0, recovered = 0;

	/* step 1: recover xattr */
	if (IS_INODE(page)) {
		recover_inline_xattr(inode, page);
	} else if (f2fs_has_xattr_block(ofs_of_node(page))) {
		/*
		 * Deprecated; xattr blocks should be found from cold log.
		 * But, we should remain this for backward compatibility.
		 */
		recover_xattr_data(inode, page, blkaddr);
		goto out;
	}

	/* step 2: recover inline data */
	if (recover_inline_data(inode, page))
		goto out;

	/* step 3: recover data indices */
<<<<<<< HEAD
	start = start_bidx_of_node(ofs_of_node(page), fi);
	end = start + ADDRS_PER_PAGE(page, fi);

	f2fs_lock_op(sbi);
=======
	start = start_bidx_of_node(ofs_of_node(page), inode);
	end = start + ADDRS_PER_PAGE(page, inode);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

	set_new_dnode(&dn, inode, NULL, NULL, 0);

	err = get_dnode_of_data(&dn, start, ALLOC_NODE);
<<<<<<< HEAD
	if (err) {
		f2fs_unlock_op(sbi);
		goto out;
	}

	f2fs_wait_on_page_writeback(dn.node_page, NODE);
=======
	if (err)
		goto out;

	f2fs_wait_on_page_writeback(dn.node_page, NODE, true);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

	get_node_info(sbi, dn.nid, &ni);
	f2fs_bug_on(sbi, ni.ino != ino_of_node(page));
	f2fs_bug_on(sbi, ofs_of_node(dn.node_page) != ofs_of_node(page));

<<<<<<< HEAD
	for (; start < end; start++) {
=======
	for (; start < end; start++, dn.ofs_in_node++) {
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
		block_t src, dest;

		src = datablock_addr(dn.node_page, dn.ofs_in_node);
		dest = datablock_addr(page, dn.ofs_in_node);

<<<<<<< HEAD
		if (src != dest && dest != NEW_ADDR && dest != NULL_ADDR &&
			dest >= MAIN_BLKADDR(sbi) && dest < MAX_BLKADDR(sbi)) {

			if (src == NULL_ADDR) {
				err = reserve_new_block(&dn);
				/* We should not get -ENOSPC */
				f2fs_bug_on(sbi, err);
=======
		/* skip recovering if dest is the same as src */
		if (src == dest)
			continue;

		/* dest is invalid, just invalidate src block */
		if (dest == NULL_ADDR) {
			truncate_data_blocks_range(&dn, 1);
			continue;
		}

		if ((start + 1) << PAGE_SHIFT > i_size_read(inode))
			f2fs_i_size_write(inode, (start + 1) << PAGE_SHIFT);

		/*
		 * dest is reserved block, invalidate src block
		 * and then reserve one new block in dnode page.
		 */
		if (dest == NEW_ADDR) {
			truncate_data_blocks_range(&dn, 1);
			reserve_new_block(&dn);
			continue;
		}

		/* dest is valid block, try to recover from src to dest */
		if (is_valid_blkaddr(sbi, dest, META_POR)) {

			if (src == NULL_ADDR) {
				err = reserve_new_block(&dn);
#ifdef CONFIG_F2FS_FAULT_INJECTION
				while (err)
					err = reserve_new_block(&dn);
#endif
				/* We should not get -ENOSPC */
				f2fs_bug_on(sbi, err);
				if (err)
					goto err;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
			}

			/* Check the previous node page having this index */
			err = check_index_in_prev_nodes(sbi, dest, &dn);
			if (err)
				goto err;

<<<<<<< HEAD
			set_summary(&sum, dn.nid, dn.ofs_in_node, ni.version);

			/* write dummy data page */
			recover_data_page(sbi, NULL, &sum, src, dest);
			dn.data_blkaddr = dest;
			set_data_blkaddr(&dn);
			f2fs_update_extent_cache(&dn);
			recovered++;
		}
		dn.ofs_in_node++;
	}

	if (IS_INODE(dn.node_page))
		sync_inode_page(&dn);

=======
			/* write dummy data page */
			f2fs_replace_block(sbi, &dn, src, dest,
						ni.version, false, false);
			recovered++;
		}
	}

>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	copy_node_footer(dn.node_page, page);
	fill_node_footer(dn.node_page, dn.nid, ni.ino,
					ofs_of_node(page), false);
	set_page_dirty(dn.node_page);
err:
	f2fs_put_dnode(&dn);
<<<<<<< HEAD
	f2fs_unlock_op(sbi);
=======
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
out:
	f2fs_msg(sbi->sb, KERN_NOTICE,
		"recover_data: ino = %lx, recovered = %d blocks, err = %d",
		inode->i_ino, recovered, err);
	return err;
}

<<<<<<< HEAD
static int recover_data(struct f2fs_sb_info *sbi,
				struct list_head *head, int type)
=======
static int recover_data(struct f2fs_sb_info *sbi, struct list_head *inode_list,
						struct list_head *dir_list)
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
{
	unsigned long long cp_ver = cur_cp_version(F2FS_CKPT(sbi));
	struct curseg_info *curseg;
	struct page *page = NULL;
	int err = 0;
	block_t blkaddr;

	/* get node pages in the current segment */
<<<<<<< HEAD
	curseg = CURSEG_I(sbi, type);
=======
	curseg = CURSEG_I(sbi, CURSEG_WARM_NODE);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	blkaddr = NEXT_FREE_BLKADDR(sbi, curseg);

	while (1) {
		struct fsync_inode_entry *entry;

<<<<<<< HEAD
		if (blkaddr < MAIN_BLKADDR(sbi) || blkaddr >= MAX_BLKADDR(sbi))
=======
		if (!is_valid_blkaddr(sbi, blkaddr, META_POR))
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
			break;

		ra_meta_pages_cond(sbi, blkaddr);

<<<<<<< HEAD
		page = get_meta_page(sbi, blkaddr);
=======
		page = get_tmp_page(sbi, blkaddr);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

		if (cp_ver != cpver_of_node(page)) {
			f2fs_put_page(page, 1);
			break;
		}

<<<<<<< HEAD
		entry = get_fsync_inode(head, ino_of_node(page));
=======
		entry = get_fsync_inode(inode_list, ino_of_node(page));
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
		if (!entry)
			goto next;
		/*
		 * inode(x) | CP | inode(x) | dnode(F)
		 * In this case, we can lose the latest inode(x).
		 * So, call recover_inode for the inode update.
		 */
<<<<<<< HEAD
		if (entry->last_inode == blkaddr)
			recover_inode(entry->inode, page);
		if (entry->last_dentry == blkaddr) {
			err = recover_dentry(entry->inode, page);
=======
		if (IS_INODE(page))
			recover_inode(entry->inode, page);
		if (entry->last_dentry == blkaddr) {
			err = recover_dentry(entry->inode, page, dir_list);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
			if (err) {
				f2fs_put_page(page, 1);
				break;
			}
		}
		err = do_recover_data(sbi, entry->inode, page, blkaddr);
		if (err) {
			f2fs_put_page(page, 1);
			break;
		}

<<<<<<< HEAD
		if (entry->blkaddr == blkaddr) {
			iput(entry->inode);
			list_del(&entry->list);
			kmem_cache_free(fsync_entry_slab, entry);
		}
=======
		if (entry->blkaddr == blkaddr)
			del_fsync_inode(entry);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
next:
		/* check next segment */
		blkaddr = next_blkaddr_of_node(page);
		f2fs_put_page(page, 1);
	}
	if (!err)
		allocate_new_segments(sbi);
	return err;
}

<<<<<<< HEAD
int recover_fsync_data(struct f2fs_sb_info *sbi)
{
	struct curseg_info *curseg = CURSEG_I(sbi, CURSEG_WARM_NODE);
	struct list_head inode_list;
	block_t blkaddr;
	int err;
=======
int recover_fsync_data(struct f2fs_sb_info *sbi, bool check_only)
{
	struct curseg_info *curseg = CURSEG_I(sbi, CURSEG_WARM_NODE);
	struct list_head inode_list;
	struct list_head dir_list;
	block_t blkaddr;
	int err;
	int ret = 0;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	bool need_writecp = false;

	fsync_entry_slab = f2fs_kmem_cache_create("f2fs_fsync_inode_entry",
			sizeof(struct fsync_inode_entry));
	if (!fsync_entry_slab)
		return -ENOMEM;

	INIT_LIST_HEAD(&inode_list);
<<<<<<< HEAD

	/* step #1: find fsynced inode numbers */
	set_sbi_flag(sbi, SBI_POR_DOING);
=======
	INIT_LIST_HEAD(&dir_list);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

	/* prevent checkpoint */
	mutex_lock(&sbi->cp_mutex);

	blkaddr = NEXT_FREE_BLKADDR(sbi, curseg);

<<<<<<< HEAD
	err = find_fsync_dnodes(sbi, &inode_list);
	if (err)
		goto out;

	if (list_empty(&inode_list))
		goto out;
=======
	/* step #1: find fsynced inode numbers */
	err = find_fsync_dnodes(sbi, &inode_list);
	if (err || list_empty(&inode_list))
		goto out;

	if (check_only) {
		ret = 1;
		goto out;
	}
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

	need_writecp = true;

	/* step #2: recover data */
<<<<<<< HEAD
	err = recover_data(sbi, &inode_list, CURSEG_WARM_NODE);
=======
	err = recover_data(sbi, &inode_list, &dir_list);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
	if (!err)
		f2fs_bug_on(sbi, !list_empty(&inode_list));
out:
	destroy_fsync_dnodes(&inode_list);
<<<<<<< HEAD
	kmem_cache_destroy(fsync_entry_slab);

	/* truncate meta pages to be used by the recovery */
	truncate_inode_pages_range(META_MAPPING(sbi),
			MAIN_BLKADDR(sbi) << PAGE_CACHE_SHIFT, -1);
=======

	/* truncate meta pages to be used by the recovery */
	truncate_inode_pages_range(META_MAPPING(sbi),
			(loff_t)MAIN_BLKADDR(sbi) << PAGE_SHIFT, -1);
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

	if (err) {
		truncate_inode_pages(NODE_MAPPING(sbi), 0);
		truncate_inode_pages(META_MAPPING(sbi), 0);
	}

	clear_sbi_flag(sbi, SBI_POR_DOING);
	if (err) {
<<<<<<< HEAD
		discard_next_dnode(sbi, blkaddr);
=======
		bool invalidate = false;

		if (test_opt(sbi, LFS)) {
			update_meta_page(sbi, NULL, blkaddr);
			invalidate = true;
		} else if (discard_next_dnode(sbi, blkaddr)) {
			invalidate = true;
		}
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y

		/* Flush all the NAT/SIT pages */
		while (get_pages(sbi, F2FS_DIRTY_META))
			sync_meta_pages(sbi, META, LONG_MAX);
<<<<<<< HEAD
=======

		/* invalidate temporary meta page */
		if (invalidate)
			invalidate_mapping_pages(META_MAPPING(sbi),
							blkaddr, blkaddr);

>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
		set_ckpt_flags(sbi->ckpt, CP_ERROR_FLAG);
		mutex_unlock(&sbi->cp_mutex);
	} else if (need_writecp) {
		struct cp_control cpc = {
			.reason = CP_RECOVERY,
		};
		mutex_unlock(&sbi->cp_mutex);
<<<<<<< HEAD
		write_checkpoint(sbi, &cpc);
	} else {
		mutex_unlock(&sbi->cp_mutex);
	}
	return err;
=======
		err = write_checkpoint(sbi, &cpc);
	} else {
		mutex_unlock(&sbi->cp_mutex);
	}

	destroy_fsync_dnodes(&dir_list);
	kmem_cache_destroy(fsync_entry_slab);
	return ret ? ret: err;
>>>>>>> 788b059... f2fs: Sync with upstream f2fs-stable 3.4.y
}
