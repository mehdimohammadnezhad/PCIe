/*
 * SIS8300 mmap syscall handling.
 */

#ifndef SIS8300_MMAP_H_
#define SIS8300_MMAP_H_

int sis8300_mmap(struct file *filp, struct vm_area_struct *vma);

#endif /* SIS8300_MMAP_H_ */
