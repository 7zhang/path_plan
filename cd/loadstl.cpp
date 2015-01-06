#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "loadstl.h"
#include "cd_macro.h"

int loadstl(const char *path, stldata *pstldata)
{
	FILE *file;
	if((file = fopen(path, "rb")) == NULL) {
		printf("open file error\n");
		return 1;
	}

	if(fread(&(pstldata->modelname), sizeof(char), 80, file) != 80)	{
		printf("modelname read error\n");
		return 1;
	}
	if(fread(&(pstldata->num), sizeof(int), 1, file) !=1) {
		printf("number read error\n");
		return 1;
	}

	fseek(file, 0, SEEK_END);
	int length = ftell(file);

	if((length - 84) / 50 != pstldata->num) {
//		pstldata->num = (length - 84) / 50;
#ifdef DEBUG
		printf("Error:inconsistent binary STL file, possibly in ASCII format\n");
#endif
		return 1;
	}

	
	fseek(file, 84, SEEK_SET);

	pstldata->ptriangle=(triangle *)malloc(pstldata->num * sizeof(triangle));
	if(pstldata->ptriangle == NULL) {
#ifdef DEBUG
		printf("Error:malloc error\n");
#endif
		return 1;
	}
	triangle *iter = pstldata->ptriangle;
	
	int i;
	float tmp;
	unsigned short tmp1;
	
	for(i = 0; i < pstldata->num; i++) {
		fread(&(tmp), sizeof(float), 1, file);
		iter->normalvector.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->normalvector.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->normalvector.z = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex1.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex1.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex1.z = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex2.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex2.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex2.z = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex3.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex3.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex3.z = tmp;
		fread(&(tmp1), sizeof(unsigned short), 1, file);
//		iter->attr = tmp;
		iter->parent = i;
		
		iter++;
	}

	//如果还没有到达文件末尾，则返回出错
	if (fread(&(tmp), 1, 1, file) == 1) {
#ifdef DEBUG
		printf("Error:inconsistent binary STL file, possibly in ASCII format\n");
#endif
		fclose(file);
		return 1;
	}

	fclose(file);
	return 0;
}


int writestl(const char *path, stldata stl[])
	{
	FILE *file;
	if((file = fopen(path, "wb")) == NULL) {
		printf("open file error\n");
		return 1;
	}
	if(fwrite(&(stl->modelname), sizeof(char), 80, file) != 80)	{
		printf("modelname read error\n");
		return 1;
	}
	if(fwrite(&(stl->num), 4, 1, file) !=1) {
		printf("number read error\n");
		return 1;
	}

	triangle *iter = stl->ptriangle;
	
	int i;
	float tmp;
	unsigned short tmp1;

	for(i = 0; i < stl->num; i++) {
		tmp = iter->normalvector.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->normalvector.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->normalvector.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex1.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex1.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex1.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex2.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex2.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex2.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex3.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex3.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex3.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp1 = 0/*iter->attr*/;
		fwrite(&(tmp1), sizeof(unsigned short), 1, file);
		
		iter++;
	}
	return 0;
}

int stl_check(stldata *stl)
{
	for(int i = 0; i < stl->num; ++i) {		
		double length;
		vectordot(&stl->ptriangle[i].normalvector, &stl->ptriangle[i].normalvector, &length);
		length = sqrt(length);
		if(length < 1 - MYZERO || length > 1 + MYZERO) {
			return 1;
		}

		vector3d diff1, diff2;
		double dot1, dot2;

		vectorminus(&stl->ptriangle[i].vertex1, &stl->ptriangle[i].vertex2, &diff1);
		vectorminus(&stl->ptriangle[i].vertex1, &stl->ptriangle[i].vertex3, &diff2);
		vectordot(&diff1, &stl->ptriangle[i].normalvector, &dot1);
		vectordot(&diff2, &stl->ptriangle[i].normalvector, &dot2);
		if(fabs(dot1) > 1 || fabs(dot2) > 1) {
#ifdef DEBUG
			printf("\nError: inconsistant stl\n");
#endif
			return 1;
		}
	}

	return 0;
}


