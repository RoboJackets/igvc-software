
/* matrix utilities used in video orbits */
#include "mat_util.h"
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <string.h>
#include <assert.h>

#ifdef IRIX
#include <varargs.h>
#endif /* IRIX */


/* The following frees the memory for deadThing (which must be a Var!) */
void Var_Destructor( void * deadThing ) {

    Var * deadVar = (Var * )deadThing;
    if ( deadVar == NULL )
        return;
    if (deadVar->data != NULL)
        free( deadVar->data );
    if (deadVar->image != NULL)
        free( deadVar->image );

}

/* ====================================================================== */
void transpose_matrix(const Var * A, Var * ANS)
{
    int indexA = 0,
                 indexANS;
    int x, y, chnl;
    const int chnls = A->channels;

#ifdef _DEBUG
    printf("  MATRIX TRANSPOSE\n");
#endif
    if ( ANS->M != A->N || ANS->N != A->M ) {
        fprintf( stderr, "ANS in transpose_matrix must be %d by %d, not %d by %d", A->N, A->M, ANS->M, ANS->N );
        exit(1);
    }

    for (y=0; y < A->M; y++) {
        for (x = 0; x < A->N; x++ ) {
            indexANS = (x*A->M + y) * chnls;
            for ( chnl = 0; chnl < chnls; chnl++, indexA++, indexANS++ )
                ANS->data[ indexANS ] = A->data[ indexA ];
        }
    }
}


/* ====================================================================== */
void matrix_multiply(const Var * A, const Var * B, Var * Out)
{
    int m, n;
    int i;

    /* - - check sizes - - */
    if (A->N != B->M)
    {
        printf("\nERROR: matrix multiply\nA->N != B->M\nExit!");
        exit(1);
    }
    if (  (Out->M != A->M) || (Out->N != B->N)  )
    {
        printf("\nERROR: matrix multiply\nOutput matrix badly sized\nExit!");
        exit(1);
    }
    for (m=0; m < A->M; m++)
    {
        for (n=0; n < B->N; n++)
        {
            for (i=0; i < A->N; i++)
                Out->data[m*Out->N + n] = Out->data[m*Out->N + n] +
                                          (A->data[m*A->N + i ]  *
                                           B->data[i*B->N + n]);
        }
    }
}


/* ====================================================================== */
void sub_matrix(const Var * A, const Var * B, Var *ANS)
{
    unsigned int index = 0,
                         index2 = 0;
    int x, y;
    const int chnls = A->channels;
    const int Mans = ANS->M;
    const int Nans = ANS->N;
    int M = A->M,
            N = A->N;
#if DEBUG
    printf("  MATRIX SUB\n");
#endif

    if (M != B->M || N != B->N)
        printf("  Possible ERROR (mat_util.c): Subtracting matrices of different sizes.\n(E1 is %d by %d; E2 is %d by %d)\n", M, N, B->M, B->N );
    if (Mans < M)
    {
        /*printf("  ANS contains clipped matrix subtraction in M.\n");*/
        M = Mans;
    }
    if (Nans < N)
    {
        /*printf("  ANS contains clipped matrix subtraction in N.\n");*/
        N = Nans;
    }

    if (A->var_type == INTERNAL_VARIABLE) {
        for (y=0; y < M; y++) {
            for (x=0; x < (N*chnls); x++, index++, index2++) {
                if ((A->data[index2] != NaNData) &&
                        (B->data[index2] != NaNData))
                {
                    ANS->data[index] = A->data[index2] - B->data[index2];

                    if (ANS->data[index] == NaNData)
                        ANS->data[index] = NaNData - 1;
                }
                else
                    ANS->data[index] = NaNData;
            }
            index2 += (chnls * (A->N - N));
        }
    }
    else  {
        for (y=0; y < M; y++) {
            for (x=0; x < (N*chnls); x++, index++, index2++) {
                if ((A->image[index2] != NaNImage)&&(B->image[index2] != NaNImage))
                {
                    ANS->data[index] =
                        (Image)A->image[index2] - (Image)B->image[index2];

                    if (ANS->data[index] == NaNImage)
                        ANS->data[index] = NaNImage - 1;
                }
                else
                    ANS->data[index] = NaNImage;

                /*   printf("Answer: %6g  A: %6g  B: %6g\n", A->image[index2], B->image[index2], ANS->data[index]); */
            }
            index2 += (chnls * (A->N - N));
        }
    }
}



/* ====================================================================== */
void add_matrix(const Var * A, const Var * B, Var *ANS)
{
    unsigned int index, index2;
    int x, y;
    int chnls;
    int Ma, Na, Mb, Nb, Mans, Nans;
    int M, N;


#if DEBUG
    printf("  MATRIX ADD\n");
#endif

    chnls = A->channels;

    Ma = A->M;
    Na = A->N;
    Mb = B->M;
    Nb = B->N;
    Mans = ANS->M;
    Nans = ANS->N;

    if (Ma != Mb || Na != Nb)
        printf("  Possible ERROR: Adding matrices of different sizes.\n");
    if (Mans < Ma)
    {
        /*printf("  ANS contains clipped matrix subtraction in M.\n");*/
        M = Mans;
    }
    else
        M = A->M;
    if (Nans < Na)
    {
        /*printf("  ANS contains clipped matrix subtraction in N.\n");*/
        N = Nans;
    }
    else
        N = A->N;

    index = 0;
    index2 = 0;
    if (A->var_type == INTERNAL_VARIABLE)
    {
        for (y=0; y < M; y++)
        {
            for (x=0; x < (N*chnls); x++, index++, index2++)
            {
                if ((A->data[index2] != NaNData) &&
                        (B->data[index2] != NaNData))
                {
                    ANS->data[index] = A->data[index2] + B->data[index2];
                    if (ANS->data[index] == NaNData)
                        ANS->data[index] = NaNData - 1;
                }
                else
                    ANS->data[index] = NaNData;
            }
            index2 += (chnls * (A->N - N));
        }
    }
    else if (A->var_type == IMAGE)
    {
        for (y=0; y < M; y++)
        {
            for (x=0; x < (N*chnls); x++, index++, index2++)
            {
                if ((A->image[index2] != NaNImage)&&(B->image[index2] != NaNImage))
                {
                    ANS->data[index] =
                        (Data)A->image[index2] + (Data)B->image[index2];
                    if (ANS->data[index] == NaNData)
                        ANS->data[index] = NaNData - 1;
                }
                else
                    ANS->data[index] = NaNData;
            }
            index2 += (chnls * (A->N - N));
        }
    }
}


/* ====================================================================== */
int sub_rows(const Var * A, Var * ANS)

/* Given an N*M matrix, returns an (N-1)*(M-1) matrix ANS.
  The m-th row of ANS equals (m+1)st row of A minus the mth row of A .
  Why the result is (N-1)*(M-1) and not N*(M-1) I'm not sure. */
{
    int index = 0,
                index2 = 0,
                         x, y;
    const int M = A->M;
    const int N = A->N;
    const int chnls = A->channels;

#if DEBUG
    printf("  MATRIX sub_rows\n");
#endif

    if ( ANS->M != M - 1 || ANS->N != N - 1 ) {
        fprintf( stderr, "ERROR (sub_rows): ANS is %d by %d, but A is %d by %d", ANS->M, ANS->N, A->M, A->N );
        exit(1);
    }

    if (A->var_type == INTERNAL_VARIABLE)
    {
        for (y = 0; y < (M-1); y++)
        {
            for (x=0; x < ((N-1)*chnls); x++, index++, index2++)
            {
                if ( (A->data[index2 + (N * chnls)] != NaNData) &&
                        (A->data[index2] != NaNData) )
                {
                    ANS->data[index] = A->data[index2 + (N * chnls)] -
                                       A->data[index2];
                    if (ANS->data[index] == NaNData)
                        ANS->data[index] = NaNData - 1;

                }
                else
                    ANS->data[index] = NaNData;
            }
            index2+= chnls;
        }
    }
    else if (A->var_type == IMAGE)
    {
        for (y = 0; y < (M-1); y++)
        {
            for (x=0; x < ((N-1) * chnls); x++, index++, index2++)
            {
                if ( (A->image[index2 + (N * chnls)] != NaNImage) &&
                        (A->image[index2] != NaNImage) )
                {
                    ANS->data[index] = (Data)A->image[index2 + (N * chnls)] -
                                       (Data)A->image[index2];
                    if (ANS->data[index] == NaNData)
                        ANS->data[index] = NaNData - 1;
                }
                else
                    ANS->data[index] = NaNData;
            }
            index2+= chnls;
        }
    }

    return(1);
}


/* ====================================================================== */
int sub_cols(const Var * A, Var * ANS)
{
    int index,index2, x, y;
    int M,N;
    int chnls;

#if DEBUG
    printf("  MATRIX sub_cols\n");
#endif

    M = A->M;
    N = A->N;

    ANS->M = M - 1;
    ANS->N = N - 1;
    chnls = A->channels;

    if (A->var_type == INTERNAL_VARIABLE)
    {
        index = 0;
        index2 = 0;
        for (y = 0; y < (M-1); y++)
        {
            for (x=0; x < (N-1) * chnls; x++, index++, index2++)
            {
                if ( (A->data[index2 + chnls] != NaNData) &&
                        (A->data[index2] != NaNData) )
                {
                    ANS->data[index] = A->data[index2 + chnls] -
                                       A->data[index2];
                    if (ANS->data[index] == NaNData)
                        ANS->data[index] = NaNData - 1;
                }
                else
                    ANS->data[index] = NaNData;
            }
            index2+= chnls;
        }
    }
    else if (A->var_type == IMAGE)
    {
        index = 0;
        index2 = 0;
        for (y = 0; y < (M - 1); y++)
        {
            for (x=0; x < (N-1) * chnls; x++, index++, index2++)
            {
                if ( (A->image[index2 + chnls] != NaNImage) &&
                        (A->image[index2] != NaNImage) )
                {
                    ANS->data[index] = (Data)A->image[index2 + chnls] -
                                       (Data)A->image[index2];
                    if (ANS->data[index] == NaNData)
                        ANS->data[index] = NaNData - 1;
                }
                else
                    ANS->data[index] = NaNData;
            }
            index2 += chnls;
        }
    }
    return(1);
}



/* ====================================================================== */
/* divides quantity in data by a Data type constant */
int div_mat_by_constant(const Var * A, Data constant, Var * ANS)
{
    unsigned int index;
    int chnls;

    chnls = A->channels;

    /* !!!!!!!!!!! branch on channels if color else grey */
    for (index = 0; index < (A->M * A->N * chnls); index++)
    {
        if (A->data[index] != NaNData)
            ANS->data[index] = A->data[index] / constant;
        else
            ANS->data[index] = NaNData;
    }
    return(1);
}

/* ======================================================================= */
int sum2nan(Var * A)
{
    int Rsum, Gsum, Bsum;
    int index;

    Rsum = Gsum = Bsum = 0;

    for (index = 0; index < ((A->M * A->N) -1) ; index+=3)
    {
        Rsum += (int)(A->data[index]);
        Gsum += (int)A->data[index + 1];
        Bsum += (int)A->data[index + 2];
    }
    return(Gsum);
}

/* Shouldn't check for NaNs ============================================== */
int chval(Var * A, double from, double to)
{
    int index;
    const int chnls = A->channels;
    const int M = A->M;
    const int N = A->N;

    /* This check was just paranoid:
      if(chnls < 1)
        {
          chnls = 1;
          printf("  Defaulted chnls in chval to %d\n", chnls);
        }
    */
    if (A->var_type == INTERNAL_VARIABLE) {
        for (index=0; index < M * N * chnls; index++) {
            if ( (A->data[index]) == from )
                A->data[index] = (Data) to;
        }
    } else {
        for (index=0; index < M * N * chnls; index++) {
            if ( (A->image[index]) == from )
                A->image[index] = to;
        }
    }
    return(1);
}

/* !!!! no chnls =========================================================== */
double vo_min(Var * A)
{
    unsigned int index;
    Image minchar;
    Data minData;

    /* !!!! N.B. if value of NaNImage changes this code will need to change */

    if (A->var_type == IMAGE)
    {
        minchar = A->image[0];
        for (index = 1; index < (A->M * A->N); index++)
        {
            if ( (A->image[index] < minchar) && (A->image[index] != NaNImage) )
                minchar = A->image[index];
        }
        return((double)minchar);
    }
    else if (A->var_type == INTERNAL_VARIABLE)
    {
        minData = A->data[0];
        for (index=1; index < (A->M * A->N); index++)
        {
            if ( (A->data[index] < minData) && (A->data[index] != NaNData) )
                minData = A->data[index];
        }
        return((double)minData);
    }
    return (double)0;
}


/* !!!! no chnls =========================================================== */
double vo_max(Var * A)
{
    unsigned int index;
    Image maxchar;
    Data maxData;

    /* N.B. if value of NaNImage changes this code will need to change */

    if (A->var_type == IMAGE)
    {
        if ( A->image[0] == NaNImage)
            maxchar = 0;
        else
            maxchar = A->image[0];
        for (index=1; index < (A->M * A->N); index++)
        {
            if ( (A->image[index] > maxchar) && (A->image[index] != NaNImage) )
                maxchar = A->image[index];
        }
        return( (double)maxchar );
    }
    else if (A->var_type == INTERNAL_VARIABLE)
    {
        if ( A->data[0] == NaNData)
            maxData = 0.0;
        else
            maxData = A->data[0];
        for (index=1; index < (A->M * A->N); index++)
        {
            if ( (A->data[index] > maxData) && (A->data[index] != NaNData) )
                maxData = A->data[index];
        }
        return( (double)maxData );
    }
    return 0;
}


/* ===================================================================== */
/* - - returns the mean or average of all non-NaN entries - - */
#if 0
double mean(Var * A) {
    int i;
    int total_entries = 0;
    double sum = 0.0;

    if (A->var_type == INTERNAL_VARIABLE ) {
        for (i = 0; i < (A->M * A->N); i++) {
            if (A->data[i] != NaNData) {
                total_entries++;
                sum = sum + (double) A->data[i];
            }
        }
    }
    else { /* A is an image */
        for (i = 0; i < (A->M * A->N); i++) {
            if (A->image[i] != NaNImage) {
                total_entries++;
                sum = sum +  (double) A->image[i];
            }
        }
    }
    if (total_entries > 0)
        return( sum / (double) total_entries );

    return (double)0;
}
#endif


/* ===================================================================== */
/* - - takes the absolute value of A - - */
void absval(Var * A)
{
    int i;

    if (A->var_type == IMAGE)
        return;

    if (A->var_type == INTERNAL_VARIABLE)
    {
        for (i = 0; i < (A->M * A->N); i++)
        {
            if ( (A->data[i] < 0) )
                A->data[i] = -(A->data[i]);
        }
    }
}

/* ====================================================================== */
/* returns the mean of the squared values of the matrix A                 */
/* assumes that A is an INTERNAL_VARIABLE                                 */
double meansquared( const Var * A ) {
    int i;
    int total_entries = 0;
    double sum = 0.0;

#if _DEBUG
    assert( A->var_type == INTERNAL_VARIABLE );
#endif

    for (i = 0; i < (A->M * A->N); i++) {
        if (A->data[i] != NaNData) {
            total_entries++;
            sum += (double)( A->data[i] * A->data[i] );
        }
    }

    if (total_entries > 0)
        return( sum / (double) total_entries );

    return (double)0;
}

/* ====================================================================== */
#if 0
int elemental_multiply(const Var * A, const Var * B, Var * ANS)
{
    int index;
#if _DEBUG
    assert( A->M == B->M );
    assert( A->N == B->N );
#endif

    if ( A->var_type == INTERNAL_VARIABLE ) {
        for (index = 0; index < (A->M * A->N); index++) {
            if ( (A->data[index] == NaNData) || (B->data[index] == NaNData) )
                ANS->data[index] = NaNData;
            else {
                ANS->data[index] = A->data[index] * B->data[index];
                if ( ANS->data[index] == NaNData)
                    ANS->data[index] = NaNData - (Data)1;
            }
        }
    }
    else { /* A is an image of some kind */
        for (index = 0; index < (A->M * A->N); index++) {
            if ( (A->image[index] == NaNImage) || (B->image[index] == NaNImage) )
                ANS->data[index] = NaNData;
            else {
                ANS->data[index] = A->image[index] * B->image[index];
                if ( ANS->data[index] == NaNData)
                    ANS->data[index] = NaNData - (Data)1;
            }
        }
    }
    return(1);
}
#endif
#if 0
/* ====================================================================== */
/* Sums data values of type Data in Vars and returns a Data type in ANS */
/* Note that ANS must be zeroed before starting */
int multiply_and_sum(int * ip, Var * ANS,
                     const Var * A,
                     const Var * B )
{
    unsigned int index;
#ifdef _DEBUG
    assert( A->channels = B->channels = ANS->channels );
    assert( A->M = B->M = ANS->M );
    assert( A->N = B->N = ANS->N );
#endif

    if ( A->channels > GREY_SCALE) {
        for (index = 0; index < (A->M * A->N)-1 ; index+=3) {
            ANS->data[*ip] +=  A->data[index] * B->data[index];
            ANS->data[(*ip)+1] +=  A->data[index + 1] * B->data[index + 1];
            ANS->data[(*ip)+2] +=  A->data[index + 2] * B->data[index + 2];
        }
    }
    else { /* (chnls == GREY_SCALE) */
        for (index = 0; index < A->M * A->N; index++)
            ANS->data[*ip] +=  A->data[index] * B->data[index];
    }
    (*ip)+= A->channels;
    return(1);
}
#endif
/* ====================================================================== */
void gaussian_elimination(Var * A,    /* N by N column vector */
                          Var * x,    /* N dim column vector */
                          Var * b)    /* N dim column vector */
{
    int index;
    int row;
    int col;
    Data multiplier;
    int m, n;

    n = A->N;   /* sould be the same ! */
    m = A->M;

    /* !!!!!!!!! make sure A is square and x, b are both N dimensional */

    /* - - This routine destroys A and b - - */
    for (index = 0; index < n; index++)
    {
        for (row = index + 1; row < n; row++)
        {
            if (A->data[row*n + index] != 0)
            {
                multiplier = A->data[row*n +  index] /
                             A->data[index*n + index];
                for (col = index + 1; col < n; col++)
                {
                    A->data[row*n + col] -=
                        (Data)multiplier * (Data)A->data[index*n + col];
                }
                b->data[row] -= multiplier * b->data[index];
            }
        }
    }
    x->data[n-1] = b->data[n-1] / A->data[(n-1)*n + (n-1)];
    for (row = n - 2; row >= 0; row--)
    {
        for (col = row + 1, x->data[row] = 0.0; col < n; col++)
        {
            x->data[row] -= A->data[row*n + col] * x->data[col];
        }
        x->data[row] = (x->data[row] + b->data[row]) /
                       A->data[row*n + row];
    }
}

/* ====================================================================== */
void data_to_image(Var * A)
{
    int index;
    int x, y;

    index = 0;
    for (y=0; y < A->M; y++)
    {
        for (x=0; x < A->N; x++, index++)
        {
            if (A->data[index] == NaNData)
                A->image[index] = NaNImage;
            else
                A->image[index] = (Image) A->data[index];
        }
    }
}

/* ====================================================================== */
void print_var(Var * A, int M, int N)
{
    int x, y;
    int sizeM, sizeN;
    int chnls;

    chnls = A->channels;
    if (chnls == 0)
        chnls = 1;

    sizeM = A->M;
    sizeN = A->N;

    if (M > A->M)
        M = A->M;
    if (N > A->N)
        N = A->N;

    printf("\nName: %s  M:%d  N:%d  chnls: %d  Var:%d\n", A->name, A->M, A->N,
           A->channels, A->var_type);

    if (A->var_type == IMAGE)
    {
        for (y = 0; y <  M; y++)
        {
            for (x=0; x < N; x++)
            {
                printf(" %6g ", A->image[y*sizeN*chnls + x*chnls]);
            }
            printf("\n");
        }
    }
    else if (A->var_type == INTERNAL_VARIABLE) {
        for (y = 0; y <  M; y++) {
            for (x=0; x < N; x++)
                printf(" %6g ", A->data[y*sizeN*chnls+x*chnls]);
            printf("\n");
        }
    }
    printf("\n\n");
}



/* ====================================================================== */
void print(Var * A, int Mstart, int Nstart, int Mend, int Nend)
{
    unsigned int M, N;
    int x = 0, y = 0;
    int sizeM, sizeN;
    int chnls;

    chnls = A->channels;
    if (chnls <= 0)
        chnls = 1;

    sizeM = A->M;
    sizeN = A->N;

    if (Mstart > Mend)
    {
        printf("  Warning: bad M range for print\n");
        M = Mstart - Mend;
    }
    else if (Mstart < Mend)
        M = Mend - Mstart;
    else
    {
        M = 1;
        Mstart = 1;
    }

    if (Nstart > Nend)
    {
        printf("  Warning: bad N range for print\n");
        N = Nstart - Nend;
    }
    else if (Nstart < Nend)
        N = Nend - Nstart;
    else /* Nstart = Nend */
    {
        N = 1;
        Nstart = 1;
    }


    /*  printf("\nPRINT from %d, %d  to %d, %d\n", Mstart, Nstart, Mend, Nend);*/
    printf("Name: %s  M:%d  N:%d  chnls: %d  Var:%d\n", A->name, A->M, A->N,
           A->channels, A->var_type);

    if (A->var_type == IMAGE)
    {
        for (y = Mstart-1; y <  Mend; y++)
        {
            for (x= Nstart-1; x < Nend; x++)
            {
                printf(" %6g ", A->image[y*sizeN*chnls + x*chnls]);
            }
            printf("\n");
        }
    }
    else if (A->var_type == INTERNAL_VARIABLE) {
        for (y = Mstart-1; y <  Mend; y++) {
            for (x = Nstart-1; x < Nend; x++)
                printf(" %6g ", A->data[y*sizeN*chnls+x*chnls]);
            printf("\n");
        }
    }
    printf("\n\n");
}

/* ===================================================================== */
void init_var(Var * variable, const char *name,
              int M, int N, int var_type, int channels,char * lookup_table)
{
    variable->name = name;
    variable->M = M;
    variable->N = N;
    variable->channels = channels;
    variable->var_type = var_type;

    /* Will be set when file is read and only then - a hack. */
    variable->magic_number = -1;
    variable->plm_field_1 = -1;
    variable->plm_field_2 = -1;
    if (lookup_table != NULL)
        strcpy(variable->lookup_table_name,lookup_table);

    if (var_type == INTERNAL_VARIABLE) {
        variable->data = (Data *)calloc(M*N*channels, sizeof(Data));
        variable->image = NULL;
    } else {
        variable->image = (Image *)calloc(M*N*channels, sizeof(Image));
        variable->data = NULL;
    }
}


/* ===================================================================== */
/* B = A */
void copy_image(Var * A, Var* B)
{
    int index;

    if ( (B->M != A->M) || (B->N != A->N) )
        printf("WARNING: Copying matrix into variable of unequal size\n");

    for (index = 0; index < (A->M * A->N); index ++)
        B->image[index] = A->image[index];
    B->M = A->M;
    B->N = A->N;
}


/* ===================================================================== */
/* B = A */
void copy_data(Var * A, Var* B)
{
    int index;
    for (index = 0; index < (A->M * A-> N); index ++)
        B->data[index] = A->data[index];
}


/* ===================================================================== */
void clear_data(Var * A)
{
    int index;
    for (index = 0; index < (A->M * A-> N); index ++)
        A->data[index] = 0.0;
}




/* ====================================================================== */
int elemental_multiply(Var * A, Var * B, Var * ANS)
{
    unsigned int index;

    /* - - Check if matricies are the same size - - */
    if (A->var_type == IMAGE)
    {
        for (index = 0; index < (A->M * A->N); index++)
        {
            if ( (A->image[index] == NaNImage) || (B->image[index] == NaNImage) )
                ANS->data[index] = NaNData;
            else
            {
                ANS->data[index] = A->image[index] * B->image[index];
                if ( ANS->data[index] == NaNData)
                    ANS->data[index] = NaNData - (Data)1;
            }
        }
    }
    else
    {
        for (index = 0; index < (A->M * A->N); index++)
        {
            if ( (A->data[index] == NaNData) ||
                    (B->data[index] == NaNData) )
                ANS->data[index] = NaNData;
            else
            {
                ANS->data[index] = (Data)A->data[index] *
                                   (Data)B->data[index];
                if ( ANS->data[index] == NaNData)
                    ANS->data[index] = NaNData - (Data)1;
            }
        }
    }

    return(1);
}


/* ====================================================================== */
/* Sums data values of type Data in Vars and returns a Data type in ANS  */
int multiply_and_sum(int * ip, Var * ANS,
                     int nargs,     /* number of matrix to * and + */
                     Var * A,
                     Var * B,
                     ...)
{
    va_list ap;
    unsigned int index;
    Var * C, * D, * E, * F;
    int chnls;

    chnls = A->channels;

    va_start(ap, B);
    if (chnls > GREY_SCALE)
    {
        if (nargs == 2)
        {
            for (index = 0; index < ((A->M * A->N) -1) ; index+=3)
            {
                ANS->data[*ip] +=  A->data[index] *
                                   B->data[index];
                ANS->data[(*ip)+1] +=  A->data[index + 1] *
                                       B->data[index + 1];
                ANS->data[(*ip)+2] +=  A->data[index + 2] *
                                       B->data[index + 2];
            }
        }
        else
        {
            printf("  got more matricies\n");
            C = va_arg(ap, Var *);
            D = va_arg(ap, Var *);
            E = va_arg(ap, Var *);
            F = va_arg(ap, Var *);
            if (nargs == 3)
            {
                printf("  got 3 matricies\n");
                for (index = 0; index < ((A->M * A->N) -1) ; index+=3)
                {
                    ANS->data[*ip] +=  A->data[index] *
                                       B->data[index] *
                                       C->data[index];
                    ANS->data[(*ip)+1] +=  A->data[index + 1] *
                                           B->data[index + 1] *
                                           C->data[index + 1];
                    ANS->data[(*ip)+2] +=  A->data[index + 2] *
                                           B->data[index + 2] *
                                           C->data[index + 2];
                }
            }
            else if (nargs == 4)
            {
                printf("  got 4 matricies\n");
                for (index = 0; index < ((A->M * A->N) -1) ; index+=3)
                {
                    ANS->data[*ip] +=  A->data[index] *
                                       B->data[index] *
                                       C->data[index] *
                                       D->data[index];
                    ANS->data[(*ip)+1] +=  A->data[index + 1] *
                                           B->data[index + 1] *
                                           C->data[index + 1] *
                                           D->data[index + 1];
                    ANS->data[(*ip)+2] +=  A->data[index + 2] *
                                           B->data[index + 2] *
                                           C->data[index + 2] *
                                           D->data[index + 2];
                }
            }
            else if (nargs == 5)
            {
                printf("  got 5 matricies, that all\n");
                for (index = 0; index < ((A->M * A->N) -1) ; index+=3)
                {
                    ANS->data[*ip] +=  A->data[index] *
                                       B->data[index] *
                                       C->data[index] *
                                       D->data[index] *
                                       E->data[index];
                    ANS->data[(*ip)+1] +=  A->data[index + 1] *
                                           B->data[index + 1] *
                                           C->data[index + 1] *
                                           D->data[index + 1] *
                                           E->data[index + 1];
                    ANS->data[(*ip)+2] +=  A->data[index + 2] *
                                           B->data[index + 2] *
                                           C->data[index + 2] *
                                           D->data[index + 2]  *
                                           E->data[index + 2];
                }

            }
            else
            {
                printf("  got 5 matricies, that all\n");
                for (index = 0; index < ((A->M * A->N) -1) ; index+=3)
                {
                    ANS->data[*ip] +=  A->data[index] *
                                       B->data[index] *
                                       C->data[index] *
                                       D->data[index] *
                                       E->data[index] *
                                       F->data[index];
                    ANS->data[(*ip)+1] +=  A->data[index + 1] *
                                           B->data[index + 1] *
                                           C->data[index + 1] *
                                           D->data[index + 1] *
                                           E->data[index + 1] *
                                           F->data[index + 1];
                    ANS->data[(*ip)+2] +=  A->data[index + 2] *
                                           B->data[index + 2] *
                                           C->data[index + 2] *
                                           D->data[index + 2] *
                                           E->data[index + 2] *
                                           F->data[index + 2];
                }

            }
        }
        va_end(ap);
        *ip = *ip + 3;   /* increment pointer */
    }
    else if (chnls == GREY_SCALE)
    {
        if (nargs == 2)
        {
            for (index = 0; index < ((A->M) * (A->N)); index++)
            {
                ANS->data[*ip] +=  (Data)(A->data[index] *
                                          B->data[index]);
            }
        }
        else
        {
            C = va_arg(ap, Var *);
            D = va_arg(ap, Var *);
            E = va_arg(ap, Var *);
            F = va_arg(ap, Var *);
            if (nargs == 3)
            {
                for (index = 0; index < ((A->M * A->N)) ; index++)
                {
                    ANS->data[*ip] +=  (Data)(A->data[index] *
                                              B->data[index] *
                                              C->data[index]);
                }
            }
            else if (nargs == 4)
            {
                for (index = 0; index < ((A->M * A->N)) ; index++)
                {
                    ANS->data[*ip] +=  (Data)(A->data[index] *
                                              B->data[index] *
                                              C->data[index] *
                                              D->data[index]);
                }
            }
            else if (nargs == 5)
            {
                for (index = 0; index < ((A->M * A->N)) ; index++)
                {
                    ANS->data[*ip] += (Data)(A->data[index] *
                                             B->data[index] *
                                             C->data[index] *
                                             D->data[index] *
                                             E->data[index]);
                }
            }
            else
            {
                for (index = 0; index < ((A->M * A->N)) ; index++)
                {
                    ANS->data[*ip] +=  (Data)(A->data[index] *
                                              B->data[index] *
                                              C->data[index] *
                                              D->data[index] *
                                              E->data[index] *
                                              F->data[index]);
                }
            }
        }
        va_end(ap);
        *ip = *ip + 1;   /* increment pointer */
    }
    return(1);
}










