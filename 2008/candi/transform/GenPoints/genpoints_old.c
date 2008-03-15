/*
 *  genpoints.c
 *  
 *
 *  Created by Paul Foster on Wed May 09 2007.
 *	Ported from MATLAB to allow regenerating points on the fly.
 *
 *
 */

#include "genpoints.h"
#include <stdlib.h>

void genpoints(int width, int height,int divisor){
	
	/* We will assume from now on that the robot uses NTSC and thus recieves 720x480 meant for 640x480
	   to do this, we first */
	
	int col =width;
	int row =height;
	
	int xi,yi,len,r,c,i;
	double *x0, *y0,*x1, *y1,*xt, *yt;
	double d,d2,sc,xinc,yinc;
	FILE *fid;
	// TO DO: NTSC doesn't actually use square pixels, fix this. (648x480 square pix = 720x480 NTSC pix)
	
	if ((width % divisor != 0) || (height % divisor != 0)) {
		printf("genpoints: 'divisor' must be a divisor of 'width' and 'height'\n");
		return;
	}
	
	sc=FINAL_SCALE_FACTOR;
	r=row/divisor;
	c=col/divisor;
	row=r;
	col=c;
	len=r*c;
	
	//d=.9*col;                   %%"focal length" dependent on lens and resolution
	d=c*SCALE_FREE_FOCAL_LENGTH;
	
	x0=malloc(sizeof(double)*row*col);
	y0=malloc(sizeof(double)*row*col);
	x1=malloc(sizeof(double)*row*col);
	y1=malloc(sizeof(double)*row*col);
	xt=malloc(sizeof(double)*row*col);
	yt=malloc(sizeof(double)*row*col);
	
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
	 
	for(i=0; i<len;i++){
		x1[i]=tan(sqrt(x0[i]*x0[i]+y0[i]*y0[i])/d)*x0[i]*sc*d/sqrt(x0[i]*x0[i]+y0[i]*y0[i]);
		y1[i]=tan(sqrt(x0[i]*x0[i]+y0[i]*y0[i])/d)*y0[i]*sc*d/sqrt(x0[i]*x0[i]+y0[i]*y0[i]);
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
	for(xi=0;xi<(c-1);xi++){
		for(yi=0;yi<(r-1);yi++){
			
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,xt[yi*c+xi],		yt[yi*c+xi]			);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",x1[yi*c+xi],		y1[yi*c+xi]			);
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,xt[yi*c+(xi+1)],	yt[yi*c+(xi+1)]		);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",x1[yi*c+(xi+1)],	y1[yi*c+(xi+1)]		);
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,xt[(yi+1)*c+(xi+1)],yt[(yi+1)*c+(xi+1)]);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",x1[(yi+1)*c+(xi+1)],y1[(yi+1)*c+(xi+1)]);
			fprintf(fid,"\t\tglTexCoord2f(%1.8lff,%1.8lff); " ,xt[(yi+1)*c+xi],	yt[(yi+1)*c+xi]		);
			fprintf(fid,"glVertex3f(%1.8lff,%1.8lff,0);\n",x1[(yi+1)*c+xi],	y1[(yi+1)*c+xi]		);
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
	genpoints(TRANSFORM_INPUT_WIDTH, TRANSFORM_INPUT_HEIGHT, TRANSFORM_INPUT_DIVISOR);
	
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
