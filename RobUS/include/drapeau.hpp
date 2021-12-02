#ifndef _DRAPEAUX_H__
#define _DRAPEAUX_H__

// define

// public variables

// public function
drapeaux_Init(void);
drapeaux_Drop(uint8_t ServoID);
drapeaux_Lock(uint8_t ServoID);

drapeaux_DropAll();
drapeaux_LockAll();
#endif
