#ifndef _PORK_MAT_UTIL1361346616
#define _PORK_MAT_UTIL1361346616

#define INTERNAL_VARIABLE 0
#define IMAGE 1
#define INTERNAL_INT 2

#define GREY_SCALE 1
#define RGB24 3

#include<stdlib.h>
#include<stdio.h>
#include<stddef.h>

/* - - Macro for (x,y) array indexing - - */
#define XYofA(x, y, A, row_length)  A[y*row_length + x ]

/* - - Not a Number definitions - - */
#define NaNData /*DBL_MAX 1024000000.0*/ 512000.0 /*1048576.0 */
#define NaNImage /*DBL_MAX */512000.0
#define NaNChar 255 /* Still needed for printing to image files. */

/* - - Types - - */
typedef double Data;
typedef double Image;

/* - - structures - - */
struct variable
{
    const char * name;
    Data * data;     /* signed floating point values  */
    Image * image;   /* also signed floating point values */
    int magic_number;
    char lookup_table_name[255];
    int plm_field_1;
    float plm_field_2;
    int max_value;
    int channels;
    int var_type;
    int M, N;
};
typedef struct variable Var;
/* - - prototypes for mat_util.c - - */
/* These all expect to be passed Var *'s which have been properly initialized
   (e.g. with init_var) */

void transpose_matrix(const Var *, Var *);
void matrix_multiply(const Var *, const Var *, Var *);
void sub_matrix(const Var *, const Var *, Var *);
void add_matrix(const Var *, const Var *, Var *);
int sub_rows(const Var *, Var *);
int sub_cols(const Var *, Var *);
int div_mat_by_constant(const Var * A, Data constant, Var *ANS);
double meansquared( const Var * A );
void gaussian_elimination(Var *, Var *, Var *);
void data_to_image(Var *);
void data_to_image(Var *);
void print_var(Var *, int, int);
void print(Var *, int, int, int, int);
void init_var(Var *, const char * name, int, int, int, int,char *);
void Var_Destructor( void * );
void copy_image(Var *, Var *);
void copy_data(Var *, Var *);
void clear_data(Var *);

int sum2nan(Var *);
int chval(Var *, double, double);
double min(Var *);
double max(Var *);
void absval(Var *);

int multiply_and_sum(int * ip, Var * ANS,
                     int nargs,
                     Var * A,
                     Var * B,
                     ...);

int elemental_multiply(Var * A, Var * B, Var * ANS);

#endif
