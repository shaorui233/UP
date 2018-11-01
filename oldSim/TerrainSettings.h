
#ifndef __TERRAIN_SETTINGS_H__
#define __TERRAIN_SETTINGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define K_TERRAIN_MAX_LINE_LENGTH 1024
#define K_TERRAIN_MAX_DOUBLES_PER_LINE 1024

int InitGlobalTerrainInfoFromFile(const char * file_name);
int GetGroundInformationSmooth(double p[3], double * p_z, double normal[3], double * p_constant);


#ifdef __cplusplus
}
#endif

#endif


