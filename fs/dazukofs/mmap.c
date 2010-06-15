/* dazukofs: access control stackable filesystem

   Copyright (C) 1997-2003 Erez Zadok
   Copyright (C) 2001-2003 Stony Brook University
   Copyright (C) 2004-2007 International Business Machines Corp.
   Copyright (C) 2008-2010 John Ogness
     Author: John Ogness <dazukocode@ogness.net>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>

#include "dazukofs_fs.h"

static int dazukofs_writepage(struct page *page, struct writeback_control *wbc)
{
	/* mmap read-only */
	return -EINVAL;
}

/**
 * Description: Called by the VM to read a page from backing store. The page
 * will be Locked when readpage is called, and should be unlocked and marked
 * uptodate once the read completes. If ->readpage discovers that it needs
 * to unlock the page for some reason, it can do so, and then return
 * AOP_TRUNCATED_PAGE. In this case, the page will be relocated, relocked
 * and if that all succeeds, ->readpage will be called again.
 */
static int dazukofs_readpage(struct file *file, struct page *page)
{
	struct dentry *dentry = file->f_dentry;
	struct file *lower_file = get_lower_file(file);
	struct inode *inode = dentry->d_inode;
	struct inode *lower_inode = get_lower_inode(inode);
	const struct address_space_operations *lower_a_ops =
		lower_inode->i_mapping->a_ops;
	char *page_data;
	struct page *lower_page;
	char *lower_page_data;
	int err = 0;

	lower_page = read_cache_page(lower_inode->i_mapping, page->index,
				     (filler_t *)lower_a_ops->readpage,
				     (void *)lower_file);

	if (IS_ERR(lower_page)) {
		err = PTR_ERR(lower_page);
		lower_page = NULL;
		printk(KERN_ERR "dazukofs: Error reading from page cache.\n");
		goto out;
	}

	wait_on_page_locked(lower_page);

	page_data = (char *)kmap(page);
	if (!page_data) {
		err = -ENOMEM;
		printk(KERN_ERR "dazukofs: Error mapping page.\n");
		goto out;
	}

	lower_page_data = (char *)kmap(lower_page);
	if (!lower_page_data) {
		err = -ENOMEM;
		printk(KERN_ERR "dazukofs: Error mapping lower page.\n");
		goto out;
	}

	memcpy(page_data, lower_page_data, PAGE_CACHE_SIZE);

	kunmap(lower_page);
	kunmap(page);
out:
	if (lower_page)
		page_cache_release(lower_page);

	if (err)
		ClearPageUptodate(page);
	else
		SetPageUptodate(page);

	unlock_page(page);
	return err;
}

int dazukofs_write_begin(struct file *f, struct address_space *mapping,
			 loff_t pos, unsigned len, unsigned flags,
			 struct page **pagep, void **fsdata)
{
	/* mmap read-only */
	return -EINVAL;
}

/**
 * Unused operations:
 *   - sync_page
 *   - writepages
 *   - set_page_dirty
 *   - readpages
 *   - prepare_write
 *   - commit_write
 *   - write_end
 *   - bmap
 *   - invalidatepage
 *   - releasepage
 *   - direct_IO
 *   - get_xip_page
 *   - migratepage
 *   - launder_page
 */
const struct address_space_operations dazukofs_aops = {
	.writepage	= dazukofs_writepage,
	.readpage	= dazukofs_readpage,
	.write_begin	= dazukofs_write_begin,
};
