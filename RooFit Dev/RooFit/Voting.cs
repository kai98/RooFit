using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

// Import functions from RooFitUtils;
using static RooFit.Utils;

namespace RooFit
{
    class Voting
    {
        // Global Variables
        // default tolerance. 
        double tol = 0.001;

        // Inputs
        // Point Cloud
        List<Point3d> pts = new List<Point3d>();
        // Extrusion from Footprint
        Brep extrusion = new Brep();
        // Potential Planes
        List<Plane> planeList = new List<Plane>();
        // Parameter to control the reconstruction performance if applicable.
        double n = 0;

        // Data
        List<Point3d> vertex = new List<Point3d>();
        Mesh delMesh;

        // Dictionary(Plane -> Dictionary (vertex index -> distance to the plane));
        Dictionary<Plane, Dictionary<int, double>> distanceDictionary = new Dictionary<Plane, Dictionary<int, double>>();

        public Voting(List<Point3d> _pts, Brep _extrusion, List<Plane> _planes, double _n = -1, double tol = 0.001)
        {
            pts = _pts;
            extrusion = _extrusion;
            planeList = _planes;
            n = _n;
        }

        public void Solve()
        {
            // Step 1, rebuild del mesh
            delMesh = DelaunayMesh2(pts);

            foreach(Plane p in planeList)
            {

            }
        }
    }
}
