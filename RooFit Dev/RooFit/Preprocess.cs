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
    class Preprocess
    {
        double tol = 0.001;

        double heightCoefficient = 1.5;

        List<Point3d> pts = new List<Point3d>();

        double height = 0;

        Mesh delMesh = new Mesh();

        // Outputs
        public List<Point3d> ptInBox = new List<Point3d>();
        public Mesh meshInBox = new Mesh();
        public Brep extrusion = new Brep();
        public Curve ftCurve = null;

        public Preprocess(List<Point3d> _pts, Curve _footprint, double _height, double _tol=0.001)
        {
            this.pts = _pts;
            this.ftCurve = _footprint;
            this.height = _height;
            this.tol = _tol;
        }

        public void Solve()
        {
            // height input missing
            // calculate the max_height using the (tallest point - z of footprint) * coefficient
            if (height == 0)
            {
                Double footprintZ = ftCurve.PointAtStart.Z;
                Double ptMaxZ = GetMaxZ(pts);
                height = (ptMaxZ - footprintZ) * heightCoefficient;
            }

            // Footprint orientation..
            // Generate the BrepBox, test if centroid 
            Vector3d heightVector = new Vector3d(0, 0, 1) * height;
            var planar = Brep.CreatePlanarBreps(ftCurve, tol)[0];

            // planar.Faces[0].NormalAt()
            extrusion = Extrude(planar, heightVector, tol);
            extrusion = extrusion.CapPlanarHoles(tol);

            // If the planar is pointing upward, flip it. 
            // Planar as the bottom of the brep should points downward. 
            //if (!isPlanarDownward(planar))
            //    extrusion.Flip();

            // If the extrusion is not pointing outward, flip it. 
            if (extrusion.SolidOrientation != BrepSolidOrientation.Outward)
                extrusion.Flip();

            // Get the points that inside the extrusion
            ptInBox = PointsInBox(pts, extrusion);

            // If the number of valid points is less than 3. Throw an error
            if (ptInBox.Count < 3)
            {
                return;
                // throw new Exception("Number of valid point is less than 3. Mesh will not be generated");
            }
            meshInBox = GetMeshInBoundary(ptInBox, extrusion);
        }

        Mesh GetMeshInBoundary(List<Point3d> pts, Brep box)
        {
            // Only test the centroid of each meshface. 

            Mesh delMesh = DelaunayMesh2(pts);

            Mesh result = new Mesh();
            result.Vertices.AddVertices(delMesh.Vertices);
            for (int i = 0; i < delMesh.Faces.Count; i++)
            {
                // Get the center point of current face. 
                Point3d centroid = delMesh.Faces.GetFaceCenter(i);

                // if the center point is inside the box
                if (extrusion.IsPointInside(centroid, tol, true))
                    result.Faces.AddFace(delMesh.Faces[i]);
            }
            result.Compact();
            return result;
        }

        bool isPlanarDownward(Brep planar)
        {
            double u, v;
            var face = planar.Faces[0];
            face.ClosestPoint(ftCurve.PointAtStart, out u, out v);
            Vector3d planarNormal = face.NormalAt(u, v);

            return planarNormal.Z < 0;
        }
    }
}
