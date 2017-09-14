/*
 * SIS8300 llseek syscall handler.
 */

#ifndef SIS8300_LLSEEK_H_
#define SIS8300_LLSEEK_H_

loff_t sis8300_llseek(struct file *filp, loff_t offset, int origin);

#endif /* SIS8300_LLSEEK_H_ */
