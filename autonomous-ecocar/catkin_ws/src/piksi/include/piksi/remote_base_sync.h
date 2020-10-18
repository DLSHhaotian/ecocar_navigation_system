#ifndef REMOTE_BASE_SYNC_h
#define REMOTE_BASE_SYNC_h

void connectSerial();

bool remoteBaseSyncInitialize();

bool remoteBaseSyncLoop();

bool connectRemoteBase();

void closeRemoteBase();

#endif