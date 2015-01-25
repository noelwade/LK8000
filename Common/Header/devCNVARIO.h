#ifndef	DEVCNVARIO_H
#define	DEVCNVARIO_H
 


BOOL cnVarioRegister(void);
BOOL cnVarioPutBugs(PDeviceDescriptor_t d, double Bugs);
BOOL cnVarioPutMacCready(PDeviceDescriptor_t d, double MacCready);
BOOL cnVarioPutBallast(PDeviceDescriptor_t d, double Ballast);
#endif
