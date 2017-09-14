/*
 * SIS8300 ioctl syscall handling.
 */

#ifndef SIS8300_IOCTL_H_
#define SIS8300_IOCTL_H_

long sis8300_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif /* SIS8300_IOCTL_H_ */
