#ifndef GEO_H
#define GEO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include <unistd.h>


#define	eClarke1866	5		//names for ellipsoidId's
#define	eGRS80		19
#define	eWGS72		21
#define	eWGS84		22

#define	dNAD27_MB_ON	6		//names for datumId's
#define	dNAD27_Canada	15
#define	dNAD83_Canada	22
#define	dNAD83_ConUS	23
#define	dWGS84		27

namespace geo {

    class Ellipsoid{
    public:
        Ellipsoid(){};
        Ellipsoid(int id, char* name, double radius, double fr){
            Name=name;  EquatorialRadius=radius;  eccSquared=2/fr-1/(fr*fr);
        }
        char* Name;
        double EquatorialRadius;
        double eccSquared;
    };

    class Datum{
    public:
        Datum(){};
        Datum(int id, char* name, int eid, double dx, double dy, double dz){
            Name=name;  eId=eid;  dX=dx;  dY=dy;  dZ=dz;
        }
        char* Name;
        int   eId;
        double dX;
        double dY;
        double dZ;
    };

    void DatumConvert(int dIn, double LatIn, double LongIn, double HtIn, int dTo,  double& LatTo, double& LongTo, double& HtTo);
    void LLtoUTM(double Lat, double Long,  double& Northing, double& Easting, int& Zone, int eId = 22); // WGS84 by default
    void UTMtoLL(double Northing, double Easting, int Zone,  double& Lat, double& Long, int eId = 22); // WGS84 by default

};

#endif