#include <iostream> 
#include <stdio.h>
#include <cstdlib>
#include <new>
#include <cmath>
#include <graphics.h>
#include <time.h>
using namespace std;
#define OutOfView 10000
#define SIZE 2 //scaling factor
const float Pi=3.1415926;
const float lapse=0.2,tDelta=0.5; //data duration for missile data collecting
const int wx=1800, wy=1000; //視窗大小 
const int x_offset=20, y_offset=20;
int wx1=wx+2*x_offset;
int wy1=wy+2*y_offset;
float fx=3000,fy=3000,fz=1000,lFocus=fx/3; //焦點及焦距 
int color;
int tCount=0;
int rr; //rr for rotation rate
int tDuration=60, nDrop=23;
float DEV=0.1;
int *mask;

class point
{
	public:
	float x,y,z;
	point()
	{
		x=0;
		y=0;
		z=0;
		return;
	}
		
	point(float xx,float yy,float zz)
	{
		x=xx;
		y=yy;
		z=zz;
		return;
	}	
	
	float length()
	{
		return pow(x*x+y*y+z*z,0.5);
	}
};

class vector
{
	public:
	point target;
	float length;
	
	vector(point p2)
	{
		target.x=p2.x;
		target.y=p2.y;
		target.z=p2.z;
		length=pow(pow(p2.x,2)+pow(p2.y,2)+pow(p2.z,2),0.5);
		return;
	}	
	
	vector(point p1,point p2)
	{

		target.x=p2.x-p1.x;
		target.y=p2.y-p1.y;
		target.z=p2.z-p1.z;
		length=pow(pow(target.x,2)+pow(target.y,2)+pow(target.z,2),0.5);
		return;	
	}
	
	vector()
	{
		target.x=0;
		target.y=0;
		target.z=0;
		length=0;
		return;
	}	
	
	vector uv()
	{
		point p;
		p.x=(target.x)/length;
		p.y=(target.y)/length;
		p.z=(target.z)/length;
		return p;
	}
	
	vector translate(point p)
	{
		p.x+=target.x;
		p.y+=target.y;
		p.z+=target.z;
		return p;
	}
	
	vector scale(float s)
	{
		point p;
		p.x=(target.x)*s;
		p.y=(target.y)*s;
		p.z=(target.z)*s;
		return p;
	}	
	
	vector rotate(int axis,float angle)//axis=1 for x, 2 for y, 3 for z axis
	{
		point p;
		switch(axis)
		{
		case 1: 
			p.x=target.x;
			p.y=target.y*cos(angle)-target.z*sin(angle);
			p.z=target.y*sin(angle)+target.z*cos(angle);
			break;
		case 2: 
			p.y=target.y;
			p.x=target.x*cos(angle)+target.z*sin(angle);
			p.z=-target.x*sin(angle)+target.z*cos(angle);
			break;
		case 3: 
			p.z=target.z;
			p.x=target.x*cos(angle)-target.y*sin(angle);
			p.y=target.x*sin(angle)+target.y*cos(angle);
			break;		
		}
		return p;
	}
	
	
	float inProduct(point p)
	{
		return (target.x)*(p.x)+(target.y)*(p.y)+(target.z)*(p.z);
	}
	
	vector outProduct(point p)
	{
		point q;
		q.x=(target.y)*(p.z)-(target.z)*(p.y);
		q.y=(target.z)*(p.x)-(target.x)*(p.z);
		q.z=(target.x)*(p.y)-(target.y)*(p.x);
		return q;
	}
	
	float angle(point p)
	{
		float a=inProduct(p)/length/p.length();
		return acos(a);
	}
};

class view
{
	public:
	float x,y;
	float xCenter,yCenter;
	point origin;//視窗之原點3D座標 
	point vx,vy;//視窗之 X 及 Y 軸單位向量 
	point fp; //focus point 之3D座標 
	float fl; //focus length
	
	view()
	{
		fp.x=fx;
		fp.y=fy;
		fp.z=fz;
		fl=lFocus;
		xCenter=wx1/2;
		yCenter=wy1/2;
		vector vTemp1(fp);
		origin=vTemp1.uv().scale(-fl).translate(fp).target;	
		point p0(0,0,-1);
		vector vTemp2(pProj3D(p0));
		vector vTemp3(origin);
		vy=vTemp3.scale(-1).translate(vTemp2.target).uv().target;
		vx=vTemp1.outProduct(vy).uv().target;
		return;
	}
	
	view(point p,point p1,float l)//
	{
		fp.x=p.x;
		fp.y=p.y;
		fp.z=p.z;
		fl=l;
		xCenter=wx1/2;
		yCenter=wy1/2;
		vector vTemp1(fp,p1);
		origin=vTemp1.uv().scale(fl).translate(fp).target;	
		point p0(p1.x,p1.y,p1.z-1);
		vector vTemp2(pProj3D(p0));
		vector vTemp3(origin);
		vy=vTemp3.scale(-1).translate(vTemp2.target).uv().target;
		vx=vTemp1.scale(-1).outProduct(vy).uv().target;
		return;
	}
	
	point pProj3D(point p) //p點投影後之3D座標 
	{
		vector vTemp0(fp);
		vector vTemp1(fp,p);
		point p0=vTemp1.uv().scale(fl/cos(vTemp1.uv().angle(vTemp0.scale(-1).uv().target))).translate(fp).target;
		return p0;//vTemp1.uv().scale(fl/cos(vTemp1.angle(vTemp0.scale(-1).target))).translate(fp).target;
	}
	
	void Proj2D(point p) // p點投影後之視窗座標 
	{
		vector vT1(origin,p),vT2(fp,origin);
		if (vT1.inProduct(vT2.target) <= 0)
		{
			x=OutOfView;
			y=OutOfView;
			return;
		}
		point p0=pProj3D(p);
		vector vTemp1(origin,p0);
		x=xCenter+vTemp1.inProduct(vx);
		y=yCenter+vTemp1.inProduct(vy);
		return;	
	}
	
	void d3Point(point p) //視窗中畫出 P點 
	{
		point p0(0,0,0);
		vector v1(fp,p),v2(fp,p0);
		Proj2D(p);
		if (x==OutOfView) return;
		//putpixel(x,y,color);
		circle(x,y,1/(v1.length*cos(v1.angle(v2.target)))*fp.x*SIZE+1);
		//circle(x*(1+DEV*(rand()%100-50)/50),y*(1+DEV*(rand()%100-50)/50),1/(v1.length*cos(v1.angle(v2.target)))*fp.x*SIZE+1);
		return;
	}
	
	void d3LineEmit(point p,point d) //p:source point  d:direction vector 視窗中畫射線 
	{
		Proj2D(p);
		if (x==OutOfView) return;
		float xT1=x,yT1=y;
		while (x>=0 && x<wx1 && y>=0 && y<wy1)
		{
			p.x+=d.x*10;
			p.y+=d.y*10;
			p.z+=d.z*10;
			Proj2D(p);	
			
			if (x==OutOfView) return;
			putpixel(x,y,10);
		}
		return;
	}
		
	void d3LineSeg(point p1,point p2) //視窗中畫線段 
	{
		Proj2D(p1);
		if (x==OutOfView) return;
		float xT=x, yT=y;
		Proj2D(p2);
		if (x==OutOfView) return;
		float distance=pow(pow(x-xT,2)+pow(y-yT,2),0.5);
		if (x<(-distance) || xT<(-distance) || x>(wx1+distance) || xT>(wx1+distance)) return;
		if (y<(-distance) || yT<(-distance) || y>(wy1+distance) || yT>(wy1+distance)) return;
		line(x,y,xT,yT);
		return;
	}	
};

class waterdrop //水珠  
{
	public:
	point pCurrent;
	point v;
	point a;
	float g0,r0,r1,r2,n0;
	view w;
	
	
	waterdrop()
	{
		int i;
		point p(0,0,0);
		pCurrent=p;
		v=p;
		a=p;
		v.z=200;

		g0=9.8;
		r0=0.04;
		r1=0.005;
		r2=0.0003;
		n0=-1;

		return;	
	}
	waterdrop(point p1,point v1)
	{
		int i;
		point p(0,0,0);
		pCurrent=p1;
		v=v1;
		a=p;
		//v.z=200;

		g0=9.80;
		r0=0.04;
		r1=0.005;
		r2=0.0003;
		n0=-1;

		return;	
	}

	int pNext(float t) //計算 T時間後之位置 
	{
		int i;
		float r;
			if (pCurrent.z<0)
				return -1;

			r = (pow(v.length(),2)*r2 + v.length()*r1 + r0)/v.length();
			a.x=r*(-v.x)+n0*v.y/v.length();
			a.y=r*(-v.y)+n0*(-v.x)/v.length();
			a.z=r*(-v.z)-g0;
			pCurrent.x+=v.x*t+0.5*a.x*t*t;
			pCurrent.y+=v.y*t+0.5*a.y*t*t;
			pCurrent.z+=v.z*t+0.5*a.z*t*t;
			v.x=v.x+a.x*t;
			v.y=v.y+a.y*t;
			v.z=v.z+a.z*t;

		return 1;		
	}	

void show(int color) 
{
	int i;
	float width;	
	setcolor(color);		
	w.d3Point(pCurrent);	
	return;
}
};

class waterjet //水柱 
{
	public:
	waterdrop *jet;
	int *on;
	point pCurrent;
	point v;
	float angle;
	int number; //number of waterdrops
	
	waterjet()
	{
	 	int i,n=tDuration;
		jet=new waterdrop[n];
		on=new int[n];
		angle=0;
		//pCurrent=p1;
		//v=p2;
		number=n;
		for(i=0;i<n;i++)
		{
			//jet[i].pCurrent=pCurrent;
			//jet[i].v=v;
			on[i]=0;
		} 
		return;	
	}	
	
	waterjet(point p1,point p2, int n)
	{
	 	int i;
		jet=new waterdrop[n];
		on=new int[n];
		pCurrent=p1;
		v=p2;
		angle=0;
		number=n;
		for(i=0;i<n;i++)
		{
			jet[i].pCurrent=pCurrent;
			jet[i].v=v;
			on[i]=0;
		} 
		return;	
	}
	
	void span(int n) //繞 Y 軸旋轉噴水 , number/n for number of cycles
	{
		float l=v.length();

			for (int i=0;i<number;i++)
			{
				if ((i/n)%2==0)
				{
					jet[i].v.x=l*cos(Pi/n*(i%n))*cos(angle)*(1+DEV*(rand()%100-50)/50);
					jet[i].v.y=l*cos(Pi/n*(i%n))*sin(angle)*(1+DEV*(rand()%100-50)/50);			
					jet[i].v.z=l*sin(Pi/n*(i%n))*(1+DEV*(rand()%100-50)/50);
				}
				else
				{
					jet[i].v.x=l*cos(Pi/n*(n-i%n))*cos(angle)*(1+DEV*(rand()%100-50)/50);
					jet[i].v.y=l*cos(Pi/n*(n-i%n))*sin(angle)*(1+DEV*(rand()%100-50)/50);
					jet[i].v.z=l*sin(Pi/n*(n-i%n))*(1+DEV*(rand()%100-50)/50);				
				}
				//cout<<i<<":"<<jet[i].v.x<<","<<jet[i].v.z<<endl;
			}
			return;
	}
	
	void up(int n) //往上噴水 
	{
		float l=v.length();

			for (int i=0;i<number;i++)
			{
				if ((i/n)%2==0)
				{
					//jet[i].v.x=l*cos(Pi/n*(i%n))*cos(angle);
					//jet[i].v.y=l*cos(Pi/n*(i%n))*sin(angle);			
					jet[i].v.z=l*sin(Pi/n*(i%n))*(1+DEV*(rand()%100-50)/50);
				}
				else
				{
					//jet[i].v.x=l*cos(Pi/n*(n-i%n))*cos(angle);
					//jet[i].v.y=l*cos(Pi/n*(n-i%n))*sin(angle);
					jet[i].v.z=l*sin(Pi/n*(n-i%n))*(1+DEV*(rand()%100-50)/50);				
				}
			}
			return;
	}
	
	void mup(int n) //往上噴水 
	{
		//float l=v.length();

			for (int i=0;i<number;i++)
			{
				jet[i].v=v;
				if ((i/n)%2==0)
				{
					//jet[i].v.x=l*cos(Pi/n*(i%n))*cos(angle);
					//jet[i].v.y=l*cos(Pi/n*(i%n))*sin(angle);			
					jet[i].v.z*=sin(Pi/n*(i%n))*(1+DEV*(rand()%100-50)/50);
				}
				else
				{
					//jet[i].v.x=l*cos(Pi/n*(n-i%n))*cos(angle);
					//jet[i].v.y=l*cos(Pi/n*(n-i%n))*sin(angle);
					jet[i].v.z*=sin(Pi/n*(n-i%n))*(1+DEV*(rand()%100-50)/50);				
				}
			}
			return;
	}
	
	void mspan(int n)  
	{
		//float l=v.length();

			for (int i=0;i<number;i++)
			{		
				jet[i].v=v;
				if ((i/n)%2==0)
				{
					jet[i].v.x*=cos(Pi/n*(i%n))*(1+DEV*(rand()%100-50)/50);
					jet[i].v.y*=cos(Pi/n*(i%n))*(1+DEV*(rand()%100-50)/50);			
					jet[i].v.z*=sin(Pi/n*(i%n))*(1+DEV*(rand()%100-50)/50);
				}
				else
				{
					jet[i].v.x*=cos(Pi/n*(n-i%n))*(1+DEV*(rand()%100-50)/50);
					jet[i].v.y*=cos(Pi/n*(n-i%n))*(1+DEV*(rand()%100-50)/50);
					jet[i].v.z*=sin(Pi/n*(n-i%n))*(1+DEV*(rand()%100-50)/50);				
				}
	
			}
			return;
	}
	
	void swing(int n,int m,float a) //旋轉噴水 m for rotation rate Pi/m, a for evalation angle 
	{
		float l=v.length();
			for (int i=0;i<number;i++)
			{
				{
					jet[i].v.x=l*cos(a)*cos(Pi/m*i)*(1+DEV*(rand()%100-50)/50);
					jet[i].v.y=l*cos(a)*sin(Pi/m*i)*(1+DEV*(rand()%100-50)/50);			
					jet[i].v.z=l*sin(a)*(1+DEV*(rand()%100-50)/50);
				}
			}
			return;
	}
	
	void mswing(int n,int m,float a) //旋轉噴水 m for rotation rate Pi/m, a for evalation angle 
	{
		//float l=v.length();
			for (int i=0;i<number;i++)
			{
				jet[i].v=v;
				{
					jet[i].v.x*=cos(a)*cos(Pi/m*i)*(1+DEV*(rand()%100-50)/50);
					jet[i].v.y*=cos(a)*sin(Pi/m*i)*(1+DEV*(rand()%100-50)/50);			
					jet[i].v.z*=sin(a)*(1+DEV*(rand()%100-50)/50);
				}
			}
			return;
	}
	
	void show(int color)
	{
		int i;
		for(i=0;i<number;i++)
		{
			if(on[i]==1)
				jet[i].show(color);
		}
		return;
	}
	
	void next()
	{
		int i;
		for(i=0;i<number;i++)
		{
			if(on[i]==1)
			{
				int temp=jet[i].pNext(tDelta);
				if (temp==-1) on[i]=0;
			}
				
		}
		return;	
	}
};


class terrain
{
	public:
	terrain()
	{	
		srand (time(NULL));
		initwindow(wx1,wy1);
		setcolor(10);
		point p(fx,fy,fz),p1(0,0,0),d(1,0,0);
		float i;
		view w(p,p1,lFocus);
		p1.x=0;
		for (i=0;i<2*fy;i=i+200)
		{
			p1.y=i;
			w.d3LineEmit(p1,d);			
		}
		p1.y=0;
		d.x=0;
		d.y=1;
		for (i=0;i<2*fx;i=i+200)
		{
			p1.x=i;
			w.d3LineEmit(p1,d);			
		}
		p1.x=0;
		d.y=0;
		d.z=1;
		//w.d3LineEmit(p1,d);	
		return;	
	}
	~terrain()
	{	
		getch();
		closegraph();
	}
};

void test_waterdrop(int,int);
void test_waterjet(int,int,point,point,float,int);
int main()
{
	terrain t;
	point p(3000,0,0);//the leftmost waterjet position
	point v(0,0,100);//the initial velocity of water jet
	float angle=Pi/4*3;
	int a,i,j,k,n=10,r,nShow;//a for span angle 0 is over x axis and 2 is over y axis
					   //n for number of water jet per row
					   //nShow for number of patterns
					   //r for number of rows
	int mode=0;//mode of pattern, 0 for span, 1 for upward, 2 for swing, 3 for upward one after one
			   //10,11,12 for variable velocity of sin(), and 20,21,22 for variable velocity of 1-sin()
			   //30 for n in 1
			   //mask processing while mode>=100
	int maskmode=0;
	//test_waterdrop(10,10);
	FILE *fp;
	fp=fopen("show.txt","r");
	if (fp==NULL) 
	{
		cout<<"error file";
		return 1;
	}
	fscanf(fp,"%d",&nShow);
	//cout<<nShow<<endl;;
	for(i=1;i<=nShow;i++)
	{
		fscanf(fp,"%d %d %d",&mode,&n,&r);
		fscanf(fp,"%f %f %f",&p.x,&p.y,&p.z);
		fscanf(fp,"%f %f %f",&(v.x),&(v.y),&(v.z));
		maskmode=mode/100;
		mode=mode%100;
		if (maskmode>0)
		{
			if (mask!=NULL) free(mask);
			mask=new int[n*r];
			for (j=0;j<n;j++)
				for (k=0;k<r;k++)
				{
					fscanf(fp,"%d",&(mask[j*n+k]));//positive mask
					if (maskmode==2)//negative mask
						mask[j*n+k]=1-mask[j*n+k];	
				}
			//cout<<"."<<endl;
		}
		
		switch (mode)
		{
			case 0: 
			case 10:
			case 20:
			case 30:
			//case 300:
				//cout<<"1="<<mode<<endl;
				fscanf(fp,"%d",&a);
				angle=Pi/4*a;
				test_waterjet(n,r,p,v,angle,maskmode*100+mode); 
				break;
			case 1:
			case 11:
			case 21:
			case 31:
				test_waterjet(n,r,p,v,angle,maskmode*100+mode);
				break;		
			case 2: 
			case 12:
			case 22:
			case 32:
				fscanf(fp,"%d %d",&a,&rr);
				angle=Pi/18*a;
				test_waterjet(n,r,p,v,angle,maskmode*100+mode); 
				break;		
		}
		//cout<<"========>"<<i<<endl;
	}
	fclose(fp);
	return 1;
}

void test_waterdrop(int x,int y)
{
	waterdrop *f;
	int i,j,*temp;
	int m;
	m=x*y;
	f=new waterdrop[x*y];
	temp=new int[x*y];
	for (i=0;i<x;i++)
		for(j=0;j<y;j++)
		{
			(f[i*y+j]).pCurrent=point(200*i,200*j,0);
			(f[i*y+j]).v=point(0,0,200-10*i);	
			temp[i*y+j]=1;			
		}
	while(m!=0)
	{
		for(i=0;i<x;i++)
			for(j=0;j<y;j++)
			{
				if (temp[i*y+j]==1) 
					(f[i*y+j]).show(i%5+10);				
			}
			delay(100*lapse);
			
		for(i=0;i<x;i++)
			for(j=0;j<y;j++)
			{
				if (temp[i*y+j]==1)
				{
					(f[i*y+j]).show(0);			
					temp[i*y+j]=(f[i*y+j]).pNext(tDelta);	
					if (temp[i*y+j]==-1) 
					{
						m--;
					}
				}
			}
		}
  	return;
}

void test_waterjet( int n, int r,point p, point v, float angle, int mode)  
{
	waterjet *f;
	point pTemp(p.y-p.x,p.x-p.y,0),pTemp1;
	int maskmode=mode/100;
	mode=mode%100;
	float l,lDefault;
	if (n==1 || mode>=30) 
		l=0;
	else 
		l=pTemp.length()/(n-1);
	int i,j;
	f=new waterjet[n*r];
	for(i=0;i<n*r;i++)
	{
		if (l==0 || mode>=30)
			lDefault=300;
		else 
			lDefault=l;
		pTemp1.x=p.x+lDefault*(i/n)/pow(2,0.5);
		pTemp1.y=p.y+lDefault*(i/n)/pow(2,0.5);
		f[i].pCurrent.x=pTemp1.x-l*(i%n)/pow(2,0.5);
		f[i].pCurrent.y=pTemp1.y+l*(i%n)/pow(2,0.5);
		f[i].v=v; 
		
		if (mode>=30)
		{
			float a=(i%n+0.5)*Pi/(n);
			float temp=f[i].v.y*cos(a)-f[i].v.z*sin(a);
			f[i].v.z=f[i].v.y*sin(a)+f[i].v.z*cos(a);
			f[i].v.y=temp;
			temp=f[i].v.x*cos(Pi/4)-f[i].v.y*sin(Pi/4);
			f[i].v.y=f[i].v.x*sin(Pi/4)+f[i].v.y*cos(Pi/4);
			f[i].v.x=temp;
		}
		
		//int lmode=mode%300;
		if (mode>=10 && mode<20)
			f[i].v.z*=sin(Pi/n*(i%n+0.5))+0.5; 
		else if (mode>=20 && mode<30)
				f[i].v.z*=(1-sin(Pi/n*(i%n+0.5)))+0.5; 
		f[i].v.x*=(1+DEV*(rand()%100-50)/50);
		f[i].v.y*=(1+DEV*(rand()%100-50)/50);
		f[i].v.z*=(1+DEV*(rand()%100-50)/50);
				
		f[i].angle=angle;
		
		if (maskmode>0)
		{
				//	cout<<"mask["<<i<<"]="<<mask[i]<<endl;	
					if (mask[i]==0)
					{
						f[i].v.x=0;
						f[i].v.y=0;
						f[i].v.z=0;
					}
		}
	//if (maskmode==0)	
		switch(mode)
		{
			case 0:
			case 10:
			case 20:
			//case 30:
				f[i].span(nDrop);
				break;
			case 1:
			case 11:
			case 21:
			//case 31:
				f[i].up(nDrop);
				break;
			case 2:
			case 12:
			case 22:
			//case 32:
				f[i].swing(nDrop,rr,angle);
				break;	
			case 30:
				f[i].mspan(nDrop);
				break;	
			case 31:
			//case 31:
				f[i].mup(nDrop);
				break;
			case 32:
				f[i].mswing(nDrop,rr,angle);
				break;			
		}
	} 
	//getch();
	for(i=0;i<f[0].number;i++)
	{ 
    	for(j=0;j<n*r;j++)
    	{
    		//cout<<f[j].v.x<<","<<f[j].v.y<<","<<f[j].v.z<<endl;
			if (f[j].v.x==0 && f[j].v.y==0 && f[j].v.z==0) 
				f[j].on[i]=0;
			else 
				f[j].on[i]=1;
			f[j].jet[i].pCurrent=f[j].pCurrent;
			//if (mask[j]!=0)
    			f[j].show((j+15)%6+10);
    	}
    	delay(100*lapse);
    	for(j=0;j<n*r;j++)
    	{

		   	//if (mask[j]!=0) 
			f[j].show(0);
    		f[j].next();
    	}
    } 
  	return;
}





