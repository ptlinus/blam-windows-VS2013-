#pragma once

#include <math.h>
#include <stdio.h>

class devicePose
{
public:
	devicePose()	{ set(); }
	~devicePose()	{}

	double xR[9];
	double xt[3];
	//----------------------------------------------//
	void set()	{ x33_eye(xR); xt[0] = xt[1] = xt[2] = 0; }
	void setR(double xiR[])	{ memcpy(xR, xiR, 9 * sizeof(double)); }
	void setT(double xit[])	{ memcpy(xt, xit, 3 * sizeof(double)); }
	void set(double xiR[], double xit[])	{ setR(xiR); setT(xit); }
	void set(devicePose &in)	{ set(in.xR, in.xt); }
	//roll = euler_x | pitch = euler_y | yaw = euler_z
	void set(double euler_x, double euler_y, double euler_z, double tx, double ty, double tz)
	{
		FromEulerZYX(euler_x, euler_y, euler_z);
		xt[0] = tx;  xt[1] = ty;  xt[2] = tz;
	}
	void quaternion(double w, double x, double y, double z)
	{
		double a = w, b = x, c = y, d = z;
		double nq = sqrt(w*w + x*x + y*y + z*z);
		if (fabs(1 - nq) > 1e-6)
		{
			a /= nq; b /= nq; c /= nq; d /= nq;
		}

		double a2 = a * a;
		double b2 = b * b;
		double c2 = c * c;
		double d2 = d * d;

		double ab = a * b;
		double ac = a * c;
		double ad = a * d;

		double bc = b * c;
		double bd = b * d;

		double cd = c * d;

		xR[0] = a2 + b2 - c2 - d2;
		xR[1] = (2) * (bc - ad);
		xR[2] = (2) * (bd + ac);
		xR[3] = (2) * (bc + ad);
		xR[4] = a2 - b2 + c2 - d2;
		xR[5] = (2) * (cd - ab);
		xR[6] = (2) * (bd - ac);
		xR[7] = (2) * (cd + ab);
		xR[8] = a2 - b2 - c2 + d2;
	}
	//----------------------------------------------//
	inline double normT()	{ return x31_norm(xt); }
	inline double normR()	{ double et[3];	ToEulerZYX(et[0], et[1], et[2]); return x31_norm(et); }

	inline double roll() 	{ return fabs(cos(-asin(xR[6]))) > 1e-6 ? atan2(xR[7], xR[8]) : 0; }
	inline double pitch() 	{ return -asin(xR[6]); }
	inline double yaw() 	{ return fabs(cos(-asin(xR[6]))) > 1e-6 ? atan2(xR[3], xR[0]) : 0; }

	inline void FromEulerZYX(double euler_x, double euler_y, double euler_z)
	{
		double cph = cos(euler_x), sph = sin(euler_x);
		double ct = cos(euler_y), st = sin(euler_y);
		double cps = cos(euler_z), sps = sin(euler_z);

		xR[0] = ct * cps;	xR[1] = cps * st * sph - cph * sps;		xR[2] = cph * cps * st + sph * sps;
		xR[3] = ct * sps;	xR[4] = cph * cps + st * sph * sps;		xR[5] = -cps * sph + cph * st * sps;
		xR[6] = -st;		xR[7] = ct * sph;						xR[8] = ct * cph;
	}
	inline void ToEulerZYX(double &euler_x, double &euler_y, double &euler_z)
	{
		double theta = -asin(xR[6]), phi = 0, psi = 0;
		if (fabs(cos(theta)) > 1e-6)	{ phi = atan2(xR[7], xR[8]);	psi = atan2(xR[3], xR[0]); }
		euler_x = phi;	euler_y = theta; euler_z = psi;
	}
	//----------------------------------------------//
	void update(devicePose &incremental)
	{
		devicePose xtmp;	xtmp.set(incremental);
		double dt[3]; x33_by_x31(dt, xR, xtmp.xt);
		xt[0] += dt[0];	xt[1] += dt[1];	xt[2] += dt[2];

		x33_by_x33(xR, xR, xtmp.xR);
	}
	void inverse(devicePose &xin)
	{
		devicePose xtmp;	xtmp.set(xin);
		x33_trans(xR, xtmp.xR);
		double dt[3] = { -xtmp.xt[0], -xtmp.xt[1], -xtmp.xt[2] };
		x33_by_x31(xt, xR, dt);
	}
	void delta(devicePose &xfore, devicePose &xback)
	{
		double xRt1[9];	x33_trans(xRt1, xfore.xR);
		double dt[3] = { xback.xt[0] - xfore.xt[0], xback.xt[1] - xfore.xt[1], xback.xt[2] - xfore.xt[2] };
		x33_by_x31(xt, xRt1, dt);
		x33_by_x33(xR, xRt1, xback.xR);
	}
	double deltaDist(devicePose &xfore)
	{
		double xRt1[9];	x33_trans(xRt1, xfore.xR);
		double dt[3] = { xt[0] - xfore.xt[0], xt[1] - xfore.xt[1], xt[2] - xfore.xt[2] };
		double ct[3]; x33_by_x31(ct, xRt1, dt);
		return x31_norm(ct);
	}
	double dist(devicePose &xfore)
	{
		double dt[3] = { xt[0] - xfore.xt[0], xt[1] - xfore.xt[1], xt[2] - xfore.xt[2] };
		return x31_norm(dt);
	}
	//----------------------------------------------//
	void write(FILE *fp)
	{
		fprintf(fp, "%lf,%lf,%lf,", xt[0], xt[1], xt[2]);
		fprintf(fp, "%lf,%lf,%lf,", xR[0], xR[1], xR[2]);
		fprintf(fp, "%lf,%lf,%lf,", xR[3], xR[4], xR[5]);
		fprintf(fp, "%lf,%lf,%lf\r\n", xR[6], xR[7], xR[8]);
	}
	void read(FILE *fp)
	{
		fscanf(fp, "%lf,%lf,%lf,", &xt[0], &xt[1], &xt[2]);
		fscanf(fp, "%lf,%lf,%lf,", &xR[0], &xR[1], &xR[2]);
		fscanf(fp, "%lf,%lf,%lf,", &xR[3], &xR[4], &xR[5]);
		fscanf(fp, "%lf,%lf,%lf\r\n", &xR[6], &xR[7], &xR[8]);
	}
	//----------------------------------------------//
private:
	double x31_norm(double x31[])
	{
		return sqrt(x31[0] * x31[0] + x31[1] * x31[1] + x31[2] * x31[2]);
	}
	void x33_by_x31(double out[], double x33[], double x31[])
	{
		out[0] = x33[0] * x31[0] + x33[1] * x31[1] + x33[2] * x31[2];
		out[1] = x33[3] * x31[0] + x33[4] * x31[1] + x33[5] * x31[2];
		out[2] = x33[6] * x31[0] + x33[7] * x31[1] + x33[8] * x31[2];
	}
	void x33_by_x33(double out[], double a33[], double b33[])
	{
		double xtmp[9];
		xtmp[0] = a33[0] * b33[0] + a33[1] * b33[3] + a33[2] * b33[6];
		xtmp[1] = a33[0] * b33[1] + a33[1] * b33[4] + a33[2] * b33[7];
		xtmp[2] = a33[0] * b33[2] + a33[1] * b33[5] + a33[2] * b33[8];

		xtmp[3] = a33[3] * b33[0] + a33[4] * b33[3] + a33[5] * b33[6];
		xtmp[4] = a33[3] * b33[1] + a33[4] * b33[4] + a33[5] * b33[7];
		xtmp[5] = a33[3] * b33[2] + a33[4] * b33[5] + a33[5] * b33[8];

		xtmp[6] = a33[6] * b33[0] + a33[7] * b33[3] + a33[8] * b33[6];
		xtmp[7] = a33[6] * b33[1] + a33[7] * b33[4] + a33[8] * b33[7];
		xtmp[8] = a33[6] * b33[2] + a33[7] * b33[5] + a33[8] * b33[8];
		memcpy(out, xtmp, 9 * sizeof(double));
	}
	void x33_inv(double out[], double x33[])
	{
		double v0 = x33[4] * x33[8] - x33[5] * x33[7], v1 = x33[5] * x33[6] - x33[3] * x33[8], v2 = x33[3] * x33[7] - x33[4] * x33[6];
		double dDet = v0*x33[0] + v1*x33[1] + v2*x33[2];
		if (dDet == 0)	x33_eye(out);
		else
		{
			out[0] = v0 / dDet;	out[1] = (x33[2] * x33[7] - x33[1] * x33[8]) / dDet;	out[2] = (x33[1] * x33[5] - x33[2] * x33[4]) / dDet;
			out[3] = v1 / dDet;	out[4] = (x33[0] * x33[8] - x33[2] * x33[6]) / dDet;	out[5] = (x33[2] * x33[3] - x33[0] * x33[5]) / dDet;
			out[6] = v2 / dDet;	out[7] = (x33[1] * x33[6] - x33[0] * x33[7]) / dDet;	out[8] = (x33[0] * x33[4] - x33[1] * x33[3]) / dDet;
		}
	}
	void x33_trans(double out[], double x33[])
	{
		out[0] = x33[0];	out[1] = x33[3];	out[2] = x33[6];
		out[3] = x33[1];	out[4] = x33[4];	out[5] = x33[7];
		out[6] = x33[2];	out[7] = x33[5];	out[8] = x33[8];
	}
	double x33_det(double x33[])
	{
		double v0 = x33[4] * x33[8] - x33[5] * x33[7], v1 = x33[5] * x33[6] - x33[3] * x33[8], v2 = x33[3] * x33[7] - x33[4] * x33[6];
		return v0*x33[0] + v1*x33[1] + v2*x33[2];
	}
	void x33_eye(double out[])
	{
		out[0] = out[4] = out[8] = 1;
		out[1] = out[2] = out[3] = out[5] = out[6] = out[7] = 0;
	}
};
