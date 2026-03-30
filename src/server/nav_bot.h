#ifndef NAV_BOT_H
#define NAV_BOT_H

#ifdef __cplusplus
extern "C" {
#endif

void Nav_BuildForMap(void);
void Nav_Shutdown(void);
void Nav_RegisterBuiltins(void);

#ifdef __cplusplus
}
#endif

#endif
