
#include "TerrainSettings.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include <cstdio>

#include <stdio.h>

#include <stdlib.h>

using namespace std;

typedef struct struct_terrain
{
	Eigen::MatrixXd X;
	Eigen::MatrixXd Y;
	Eigen::MatrixXd Z;
	
	Eigen::MatrixXd Zx;
	Eigen::MatrixXd Zy;
	Eigen::MatrixXd Zxy;

	double x0, xF, y0, yF;
	double dx, dy;
	int n_x, n_y;
	vector< vector<  Eigen::MatrixXd * > > a_coeffs;
} TerrainInfo;

static TerrainInfo g_terrain_info;



#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief      Determines if whitespace.
 *
 * @param[in]  chr    character to check
 *
 * @return     True if whitespace, False otherwise.
 */
int IsWhitespace(char chr) {
	return chr == ' ' || chr == '\t' || chr == '\n' || chr == '\r';
}



/**
 * @brief      Scans a string for doubles
 *
 * @param[in]  str          The input string
 * @param[out] p_data       The pointer to where doubles should be stored
 * @param[in]  max_doubles  Maximum number of doubles that can fit in p_data
 *
 * @return     -1 on failure, the number of doubles scanned on success
 * 
 * Scans string ignoring any white space and processing each data field as a double
 * 
 */
int ScanStringForDoubles(const char * str, double * p_data, int max_doubles)
{
    unsigned int str_len = strlen(str);
	char substr[K_TERRAIN_MAX_LINE_LENGTH];
	int state = 0; // state 0 is looking for the start of the string. state 1 is looking for the end of the string
	
    unsigned int index = 0;		// Index in str
	int sub_index = 0;  // Index in substr
	int doubles_found = 0;

	
	while (index < strlen(str)) 
	{
		// If we are looking for the start of the string
		if (state == 0) 
		{
			if(!IsWhitespace(str[index])) 
			{
				state = 1;
			}
		} 
		// Else we try to process the characters
		if (state == 1) 
		{
			// If we reached the end of the field, try to process string double
			if(IsWhitespace(str[index])) 
			{
				substr[sub_index] = 0;
				double val;
				if(sscanf(substr,"%lf",&val) == 1) 
				{
					if(doubles_found == max_doubles)
					{
						return -1;
					}
					p_data[doubles_found] = val;
					doubles_found++;	
				}
				state = 0;
				sub_index = 0;
			}
			else 
			{
				substr[sub_index] = str[index];
				sub_index ++;		
			}
		}
		index++;
	}
	// If we have extra data at the end of the line that wasn't terminated in white space
	if( state == 1)
	{
		substr[sub_index] = 0;
		double val;
		if( sscanf(substr,"%lf",&val) == 1) 
		{
			if(doubles_found == max_doubles)
			{
				return -1;
			}
			p_data[doubles_found] = val;
			doubles_found++;	
		}
	}
	return doubles_found;
}


void GetDifferentiationIndices(int i, int i_max, int &i1, int &i2 )
{
	i1 = max(0,i-1);
	i2 = min(i+1, i_max-1);
}


/**
 * @brief      Calculates the terrain derivatives.
 */
void ComputeTerrainDerivatives()
{
	int n_x = g_terrain_info.n_x;
	int n_y = g_terrain_info.n_y;

	int i1, i2, j1, j2;
	for (int i = 0 ; i < n_x ; i++) 
	{
		GetDifferentiationIndices(i, n_x, i1, i2);
		for (int j = 0 ; j < n_y ; j++)
		{
			GetDifferentiationIndices(j, n_y, j1, j2);
			double fx1 = g_terrain_info.Z(i1,j);
			double fx2 = g_terrain_info.Z(i2,j);

			double fy1 = g_terrain_info.Z(i,j1);
			double fy2 = g_terrain_info.Z(i,j2);

			g_terrain_info.Zx(i,j) = (fx2 - fx1)/ (i2-i1) ;
			g_terrain_info.Zy(i,j) = (fy2 - fy1)/ (j2-j1) ;
		}
	}
	for (int i = 0 ; i < n_x ; i++) 
	{
		GetDifferentiationIndices(i, n_x, i1, i2);
		for (int j = 0 ; j < n_y ; j++)
		{
			GetDifferentiationIndices(j, n_y, j1, j2);
			double fx1 = g_terrain_info.Zy(i1,j);
			double fx2 = g_terrain_info.Zy(i2,j);

			double fy1 = g_terrain_info.Zx(i,j1);
			double fy2 = g_terrain_info.Zx(i,j2);

			g_terrain_info.Zxy(i,j)  = (fx2 - fx1) / (i2-i1) ;
			g_terrain_info.Zxy(i,j) += (fy2 - fy1) / (j2-j1) ;
		}
	}
	g_terrain_info.Zxy/=2;

	Eigen::MatrixXd fsToAs(4,4);
	fsToAs << 1, 0 , -3, 2, 0, 0, 3, -2, 0, 1 , -2, 1, 0, 0, -1, 1;

	Eigen::MatrixXd Z   = g_terrain_info.Z  ;
	Eigen::MatrixXd Zx  = g_terrain_info.Zx ;
	Eigen::MatrixXd Zy  = g_terrain_info.Zy ;
	Eigen::MatrixXd Zxy = g_terrain_info.Zxy;

	g_terrain_info.a_coeffs.resize( n_x );
	for (int i = 0; i < n_x-1 ; ++i)
	{
		int x0 = i, x1 = i+1;
		g_terrain_info.a_coeffs[i].resize(n_y);
		for (int j = 0 ; j < n_y-1 ; j++)
		{
			int y0 = j, y1 = j+1;
			// See https://en.wikipedia.org/wiki/Bicubic_interpolation
			// 
			g_terrain_info.a_coeffs[i][j] = new Eigen::MatrixXd(4,4);
			Eigen::MatrixXd fs(4,4);
			fs <<  Z(x0, y0) ,  Z(x0, y1),  Zy(x0, y0),  Zy(x0, y1) ,
				   Z(x1, y0) ,  Z(x1, y1),  Zy(x1, y0),  Zy(x1, y1) ,
                  Zx(x0, y0) , Zx(x0, y1), Zxy(x0, y0), Zxy(x0, y1) ,
                  Zx(x1, y0) , Zx(x1, y1), Zxy(x1, y0), Zxy(x1, y1);

            Eigen::MatrixXd tmp = fsToAs.transpose();
            *g_terrain_info.a_coeffs[i][j] =  tmp *  fs  * fsToAs;
		}
	}
}


/**
 * @brief      Initialize Global Terrain object from terrain file
 *
 * @param[in]  file_name  The name of file that contains the terrain specification
 *
 * @return     0 on success, nonzero on failure
 */
int InitGlobalTerrainInfoFromFile(const char * file_name)
{
	//////////////
	// Pass 1: Check for consistency and set sizes;
	//////////////
	

	// Check for file existence
	FILE * p_file = fopen(file_name,"r");
	if(p_file == NULL) {
		printf("Terrain file not found.\n");
		return 1;
	}

	int line_num = 0, n_y =0, n_x=0;
	char line[K_TERRAIN_MAX_LINE_LENGTH];
	int state = 0;
	double data[K_TERRAIN_MAX_DOUBLES_PER_LINE];
	int first_height_line = 0;

	// Process lines of file
	while (fgets(line, sizeof(line), p_file)) {
        line_num = line_num + 1;

        if (line[0] == '#') {
         	continue;
        }
        
        // Initial State. Looking for x0 y0
        if (state == 0) { 
        	// Process line bit by bit
        	int num_entries = ScanStringForDoubles(line, data, K_TERRAIN_MAX_DOUBLES_PER_LINE);
        	if (num_entries == 2)
        	{
        		g_terrain_info.x0 = data[0];
        		g_terrain_info.y0 = data[1];
        		state = 1;
        	}
        	else
        	{
        		printf("[Terrain File Loading] x0 y0 line (line #%d) has formatting errror\n",line_num);
        		return -1;
        	}

        } 
        // Looking for dx dy
        else if (state == 1) {
        	int num_entries = ScanStringForDoubles(line, data, K_TERRAIN_MAX_DOUBLES_PER_LINE);
        	if (num_entries == 2)
        	{
        		g_terrain_info.dx = data[0];
        		g_terrain_info.dy = data[1];
        		state = 2;
        	}
        	else
        	{
        		printf("[Terrain File Loading] dx dy line (line #%d) has formatting errror\n",line_num);
        		return -1;
        	}
        }
        // Looking for height entries
        else
        {
        	int num_entries = ScanStringForDoubles(line, data, K_TERRAIN_MAX_DOUBLES_PER_LINE);
        	if (n_x == 0)
        	{
        		if (num_entries == 0)
        		{
        			printf("[Terrain File Loading] First height line (line #%d) has no entires.\n",line_num);
        			return -1;
        		}
        		n_y = num_entries;
        		first_height_line = line_num;
        	}

        	// Make sure there are a consistent number of entires
        	if (num_entries != n_y)
        	{
        		printf("[Terrain File Loading] Height line (line #%d) has incorrect number of entires. Expected %d, got %d.\n",line_num, n_y, num_entries);
        		return -1;
        	}
        	n_x++;
        }
    }
    fclose(p_file);

    g_terrain_info.xF = g_terrain_info.x0 + g_terrain_info.dx*n_x;
	g_terrain_info.yF = g_terrain_info.y0 + g_terrain_info.dy*n_y;

	// Allocate info for the heights
	g_terrain_info.X.resize( n_x, n_y );
	g_terrain_info.Y.resize( n_x, n_y );
	g_terrain_info.Z.resize( n_x, n_y );
	
	g_terrain_info.Zx.resize( n_x, n_y );
	g_terrain_info.Zy.resize( n_x, n_y );
	g_terrain_info.Zxy.resize( n_x, n_y );

	g_terrain_info.n_x = n_x;
	g_terrain_info.n_y = n_y;

	// Pass 2: Populate
    p_file = fopen(file_name,"r");
	if(p_file == NULL) {
		printf("Terrain file not found (second pass).\n");
		return 1;
	}

	line_num = 0;
	int height_line = 0;
	double x = g_terrain_info.x0;
	// Process lines of file
	while (fgets(line, sizeof(line), p_file)) {
        line_num = line_num + 1;
        if (line[0] == '#') 
        {
         	continue;
        }
        if (line_num >= first_height_line)
        {
        	double y = g_terrain_info.y0;
        	int num_entries = ScanStringForDoubles(line, data, K_TERRAIN_MAX_DOUBLES_PER_LINE);
        	for (int i = 0; i < n_y; ++i)
        	{
        
        		g_terrain_info.X(height_line,i) = x;
        		g_terrain_info.Y(height_line,i) = y;
        		g_terrain_info.Z(height_line,i) = data[i];

        		y += g_terrain_info.dy;
        	}
        	x+= g_terrain_info.dx;
        	height_line++;
        }
    }
    fclose(p_file);

    ComputeTerrainDerivatives();
    return 0;
}






/**
 * @brief      Determines if number in the given range
 *
 * @param[in]  x     Number in question
 * @param[in]  low   Lower bound
 * @param[in]  high  Upper bound
 *
 * @return     1 if in range, 0 if not
 */
int in_range(double x, double low, double high)
{
	return x>= low && x<=high;
}

/**
 * @brief      Gets the ground information for bicubic smooth interpolation.
 *
 * @param      p           Query position
 * @param      normal      The normal vectory
 * @param      p_constant  Pointer to the constant that defines the local surface plane
 *
 * @return     0 on success, -1 on failure
 */
int GetGroundInformationSmooth(double p[3], double * p_z, double normal[3], double * p_constant)
{
	double x = p[0], y = p[1];
	int index_x = (x - g_terrain_info.x0)/g_terrain_info.dx;
	int index_y = (y - g_terrain_info.y0)/g_terrain_info.dy;

	if ( !in_range(index_x, 0, g_terrain_info.n_x-1) )
	{
		return -1;
	}
	if ( !in_range(index_y, 0, g_terrain_info.n_y-1) )
	{
		return -1;
	}
	double sx = (x - g_terrain_info.x0 - g_terrain_info.dx * index_x) / g_terrain_info.dx;
	double sy = (y - g_terrain_info.y0 - g_terrain_info.dy * index_y) / g_terrain_info.dy;

	Eigen::Vector4d x_poly, y_poly, dx_poly, dy_poly;
	x_poly<< 1, sx, sx*sx, sx*sx*sx;
	y_poly<< 1, sy, sy*sy, sy*sy*sy;

	dx_poly << 0, 1, 2*sx, 3*sx*sx;
	dy_poly << 0, 1, 2*sy, 3*sy*sy;


	Eigen::MatrixXd A = *g_terrain_info.a_coeffs[index_x][index_y];
	Eigen::MatrixXd tmp = x_poly.transpose() * A * y_poly;

	double z = tmp(0,0);
	*p_z = z;

	tmp = dx_poly.transpose() * A * y_poly;
	double dz_dsx = tmp(0,0);

	tmp = x_poly.transpose() * A * dy_poly;
	double dz_dsy = tmp(0,0);

	Eigen::Vector3d p_vec, tan_vec_1, tan_vec_2, norm_vec;

	p_vec << p[0], p[1], z;

	double dsx_dx = 1./g_terrain_info.dx;
	double dsy_dy = 1./g_terrain_info.dy;

	tan_vec_1 << 1, 0, dz_dsx * dsx_dx;
	tan_vec_2 << 0, 1, dz_dsy * dsy_dy;

	norm_vec = tan_vec_1.cross(tan_vec_2);
	norm_vec.normalize();

	normal[0] = norm_vec(0);
	normal[1] = norm_vec(1);
	normal[2] = norm_vec(2);

	*p_constant = p_vec.transpose() * norm_vec ;
	return 0;
}


/*int main()
{
	InitGlobalTerrainInfoFromFile("Terrain.txt");
	printf("Terrain initialized\n");


	cout << "X" << endl << g_terrain_info.X << endl;
	cout << "Y" << endl << g_terrain_info.Y << endl;
	cout << "Z" << endl << g_terrain_info.Z << endl;
	cout << "Zx" << endl << g_terrain_info.Zx << endl;
	cout << "Zy" << endl << g_terrain_info.Zy << endl;
	cout << "Zxy" << endl << g_terrain_info.Zxy << endl;


	double constant, z;
	double normal[3];
	double p[3] = {.3, .4, .6};

	GetGroundInformationSmooth(p, &z, normal, &constant);

	cout << "z " << z << endl;
 	cout << "normal " << normal[0] << " , " << normal[1] << " , " << normal[2] << endl;
	cout << "constant " <<  constant << endl;

}*/


#ifdef __cplusplus
}
#endif

