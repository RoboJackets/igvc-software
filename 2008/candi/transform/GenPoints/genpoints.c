/*
 *  genpoints.c
 *  
 *
 *  Created by Paul Foster on Wed May 09 2007.
 *	Ported from MATLAB to allow regenerating points on the fly.
 *
 *
 */

/*
To do our work we must cange the representation of the location of all of our data.
Thus, for every datum, we will have a group of its representations in various 
coordinate spaces.
For each point we will have an associated location in:

	Pixel Space				
	Camera Space
	Square Camera Space
	Texel Space
	Transformed Space
	
Also, this file will be as unoptimized as possible because it is a code generator
and not expected to run more than once per edit.

MATLAB code will be commented to one tab:
	//[M code..]

Comments describing the revisions to make this code systematic and comprehensible
will be unindented:

//Easier to read stuff
*/

#include "genpoints.h"
#include <stdlib.h>

struct pxl{
	double x;
	double y;
	};
typedef struct pxl pxl;

void genpoints(int divisor){
	#define width TRANSFORM_INPUT_WIDTH
	#define height TRANSFORM_INPUT_HEIGHT
	/* We will assume from now on that the robot uses NTSC and thus recieves 720x480 meant for 640x480
	   to do this, we first setup a fractional coordinate system based on 640x480 and then translate.*/
	
	int col =640;
	int row =480;
	
	int xi,yi,len,r,c,i,n;
	double *x0, *y0,*x1, *y1,*xt, *yt;
	double d,sc;
	pxl* pi;
	pxl* po;
	FILE *fid;
	// TO DO: NTSC doesn't actually use square pixels, fix this. (648x480 square pix = 720x480 NTSC pix)
	
	if ((width % divisor != 0) || (height % divisor != 0)) {
		printf("genpoints: 'divisor' must be a divisor of 'width' and 'height'\n");
		return;
	}
	
	sc=FINAL_SCALE_FACTOR/divisor;
	r=row/divisor;
	c=col/divisor;
	row=r;
	col=c;
	len=r*c;
	
	//d=.9*col;                   %%"focal length" dependent on lens and resolution
	d=c*divisor*SCALE_FREE_FOCAL_LENGTH;
	
	x0=malloc(sizeof(double)*(640/divisor+1)*(480/divisor+1));
	y0=malloc(sizeof(double)*(640/divisor+1)*(480/divisor+1));
	x1=malloc(sizeof(double)*(640/divisor+1)*(480/divisor+1));
	y1=malloc(sizeof(double)*(640/divisor+1)*(480/divisor+1));
	xt=malloc(sizeof(double)*(640/divisor+1)*(480/divisor+1));
	yt=malloc(sizeof(double)*(640/divisor+1)*(480/divisor+1));
	pi=(pxl*)malloc(sizeof(pxl)*(640/divisor+1)*(480/divisor+1));
	
	po=(pxl*)malloc(sizeof(pxl)*(640/divisor+1)*(480/divisor+1));
	double xou,xin,you,yin;
/*
 * Here we take the input pixel space, pi, and associate it with a square pixel space,
 * po, which will be used by the actual calculations. Note also that we have implicitly 
 * downsampled in such a way as to form square divisions of the output. Also, (0,0)
 * in pi represents the lower left (or upper left, depending on how the data was sent in)
 * corner of the lower left most pixel and (720,640) the upper right corner of the upper
 * right most pixel of the input image. However, (0,0) in po is the center of the output 
 * image.
 *
 */
	for(	yin=0,you=-480/2,n=0; you<=480/2;yin+=divisor,you+=divisor){
		for(xin=0,xou=-640/2;xou<=640/2;xin+=(double)720/640*(double)divisor,xou+=divisor){
			pi[n].x=xin;
			po[n].x=xou;
			pi[n].y=yin;
			po[n].y=you;
			
			n++;
		}	
	}
	/*
	//x0=(zeros(row,1)+1)*(1:col)-(col+1)/2    ;   %%define x coordinates
	for(yi=0;yi<row;yi++){
		for(xi=0;xi<col;xi++){
			x0[yi*col+xi]=xi+1-((double)(col+1))/2;//mess by cols
			}
	}
	
	//y0=((1:row)'*(zeros(1,col)+1))-(row+1)/2   ;
	for(xi=0;xi<col;xi++){
		for(yi=0;yi<row;yi++){
			y0[yi*col+xi]=yi+1-((double)(row+1))/2;//mess by rows
		}
	}*/
	for(i=0;i<n;i++){
		x0[i]=po[i].x;
		y0[i]=po[i].y;
	}
	/*
	 *	%%optimizations
	 *	d2=1/d;                     %avoid division
	 *	k0=sc*d;                    %avoid remultiplying constants
	 *	k1=sqrt(x0.^2+y0.^2);       %intermediate stuff
	 *	k2=k0./k1;                  %consolidate reciprocals
	 *	k1=k1.*d2;                  %consolidate constant multiples while avoiding memory access
	 *	%%snoitazimitpo
	 *
	 *	x1=tan(k1).*x0.*k2;             %%magic happens
	 *	y1=tan(k1).*y0.*k2;             %%more magic
	 */
	 
	for(i=0; i<n;i++){
		if(x0[i]==0 && y0[i]==0){
			x1[i]=0;
			y1[i]=0;
		}else{
			x1[i]=tan(sqrt(x0[i]*x0[i]+y0[i]*y0[i])/d)*x0[i]*sc*d/sqrt(x0[i]*x0[i]+y0[i]*y0[i]);
			y1[i]=tan(sqrt(x0[i]*x0[i]+y0[i]*y0[i])/d)*y0[i]*sc*d/sqrt(x0[i]*x0[i]+y0[i]*y0[i]);
		}	
	}
	
	/*
	 *	xt=linspace(0,1,c);
	 *	yt=linspace(0,2/3,r);
	 *	[xt,yt]=meshgrid(xt,yt);
	 */
	 
	 /* We want to find the association between input pixels in the texture
	  * and points in the [0-1] texture space.
	  * 
	  *
	  *
	  */
	 
	/* 
	xinc= (((double)width)/((double)TEX_DIM))/((double)(c-1));
	yinc=(((double)height)/((double)TEX_DIM))/((double)(r-1));
	
	for(xi=0;xi<col;xi++){
		for(yi=0;yi<row;yi++){
			xt[yi*col+xi]=xi*xinc;
		}
	}	
	
	for(xi=0;xi<col;xi++){
		for(yi=0;yi<row;yi++){
			yt[yi*col+xi]=yi*yinc;
		}
	}
	*/
	for(i=0;i<n;i++){
		xt[i]=pi[i].x/TEX_DIM;
		yt[i]=pi[i].y/TEX_DIM;
	}
	
	/*
	 *	fid = fopen('glxcode.txt','wt');
	 *	for xi=1:(c-1)
	 *		for yi=1:(r-1)
	 *
	 *			fprintf(fid,'glTexCoord2f(%1.8gf,%1.8gf); ' ,xt(yi,xi),yt(yi,xi));
	 *			fprintf(fid,'glVertex3f(%1.8gf,%1.8gf,0);\n',x1(yi,xi),y1(yi,xi));
	 *			fprintf(fid,'glTexCoord2f(%1.8gf,%1.8gf); ' ,xt(yi,xi+1),yt(yi,xi+1));
	 *			fprintf(fid,'glVertex3f(%1.8gf,%1.8gf,0);\n',x1(yi,xi+1),y1(yi,xi+1));
	 *			fprintf(fid,'glTexCoord2f(%1.8gf,%1.8gf); ' ,xt(yi+1,xi+1),yt(yi+1,xi+1));
	 *			fprintf(fid,'glVertex3f(%1.8gf,%1.8gf,0);\n',x1(yi+1,xi+1),y1(yi+1,xi+1));
	 *			fprintf(fid,'glTexCoord2f(%1.8gf,%1.8gf); ' ,xt(yi+1,xi),yt(yi+1,xi));
	 *			fprintf(fid,'glVertex3f(%1.8gf,%1.8gf,0);\n',x1(yi+1,xi),y1(yi+1,xi));
	 *		end
	 *	end
	 *	fclose(fid);
	 */
	 
	if(!(fid = fopen("getsurf.c","wt")))
		{
		printf("Error opening file: \"getsurf.c\"\n");
		}
	
	fprintf(fid,"#include <GL/glut.h>\n");
	fprintf(fid,"#include \"getsurf.h\"\n");
	fprintf(fid,"\nvoid tsurf(){\n\tglBegin(GL_QUADS);\n");
	for(xi=0;xi<c;xi++){
		for(yi=0;yi<r;yi++){
			
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,	xt[yi*(c+1)+xi],			yt[yi*(c+1)+xi]			);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",		x1[yi*(c+1)+xi],			y1[yi*(c+1)+xi]			);
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,	xt[yi*(c+1)+(xi+1)],		yt[yi*(c+1)+(xi+1)]		);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",		x1[yi*(c+1)+(xi+1)],		y1[yi*(c+1)+(xi+1)]		);
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,	xt[(yi+1)*(c+1)+(xi+1)],	yt[(yi+1)*(c+1)+(xi+1)]);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",		x1[(yi+1)*(c+1)+(xi+1)],	y1[(yi+1)*(c+1)+(xi+1)]);
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,	xt[(yi+1)*(c+1)+xi],		yt[(yi+1)*(c+1)+xi]		);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",		x1[(yi+1)*(c+1)+xi],		y1[(yi+1)*(c+1)+xi]		);
		}
	}
	fprintf(fid,"\n\tglEnd();\n}\n");
	fflush(fid);
	if(fclose(fid))
		printf("Error closing file: \"getsurf.c\"\n");
	else 
		printf("Build complete.\n");
	fflush(fid);
}

int main(){
	int status;
	printf("\nBuilding... \n");
	genpoints( TRANSFORM_INPUT_DIVISOR);
	
	printf("Compiling... \n");
	fflush(stdout);
	status=system("gcc -c getsurf.c -o ../getsurf.o -I../");
	if(status==0){
		printf("Compilation complete.\n");
		}
		else {
		printf("\nCompilation Failed!!!\nError:%d\n",status);
		}

	return 0;
}
