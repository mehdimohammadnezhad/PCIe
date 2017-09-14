/*
 * SIS8300 write syscall handler.
 */

#ifndef SIS8300_WRITE_H_
#define SIS8300_WRITE_H_

ssize_t sis8300_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp);

#endif /* SIS8300_WRITE_H_ */
