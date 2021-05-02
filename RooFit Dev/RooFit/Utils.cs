using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace RooFit
{
    class Utils
    {

        public static Double GetMaxZ(List<Point3d> ptList)
        {
            Double maxZ = ptList.Max(p => p.Z);
            return maxZ;
        }

        public static Point3d ComputeCentroid(Curve ftCurve)
        {
            // TODO: lowest point in ftCurve. 
            Point3d centroid = AreaMassProperties.Compute(ftCurve).Centroid;
            return centroid;
        }

        public static Point3d ComputeCentroid(Brep solidBrep)
        {
            Point3d centroid = AreaMassProperties.Compute(solidBrep).Centroid;
            return centroid;
        }

        public static Point3d ComputeCentroid(Mesh mesh)
        {
            Point3d centroid = AreaMassProperties.Compute(mesh).Centroid;
            return centroid;
        }

        public static Brep FtBoundingBox(Curve ftCurve, Double height, Double tol)
        {
            Vector3d heightVector = new Vector3d(0, 0, height);
            Brep extrusion = Extrude(ftCurve, heightVector, tol);
            Brep boudingBox = extrusion.CapPlanarHoles(tol);
            return boudingBox;
        }

        public static List<Point3d> PointsInBox(IEnumerable<Point3d> pts, Brep box)
        {
            var ptList = pts.Where(pt => PointInBox(pt, box));
            return SanCheck(ptList);
        }

        public static bool PointInBox(Point3d pt, Brep box)
        {
            double local_tol = 0.001;
            bool strictly = true;
            return box.IsPointInside(pt, local_tol, strictly);
        }

        public static Mesh DelaunayMesh2(List<Point3d> pts)
        {
            //convert point3d to node2
            //grasshopper requres that nodes are saved within a Node2List for Delaunay
            // pts = DeepCopy(pts);
            var nodes = new Grasshopper.Kernel.Geometry.Node2List();

            foreach (Point3d pt in pts)
            {
                nodes.Append(new Grasshopper.Kernel.Geometry.Node2(pt.X, pt.Y));
            }

            //solve Delaunay
            var delMesh2d = new Mesh();
            var delMesh3d = new Mesh();
            var faces = new List<Grasshopper.Kernel.Geometry.Delaunay.Face>();

            faces = Grasshopper.Kernel.Geometry.Delaunay.Solver.Solve_Faces(nodes, 1);

            //output
            delMesh2d = Grasshopper.Kernel.Geometry.Delaunay.Solver.Solve_Mesh(nodes, 1, ref faces);
            
            delMesh2d.Compact();
            delMesh3d.Faces.AddFaces(delMesh2d.Faces);
            delMesh3d.Vertices.AddVertices(pts);
            
            delMesh3d.Compact();
            return delMesh3d;
        }

        // return a list without null, safely convert IEnumerable to a list. 
        public static List<T> SanCheck<T>(IEnumerable<T> items)
        {
            var itemList = new List<T>(items);
            itemList.RemoveAll(item => item == null);
            return itemList;
        }

        public static List<Point3d> DeepCopy(IEnumerable<Point3d> pts)
        {
            List<Point3d> ptList = new List<Point3d>(pts);
            List<Point3d> newList = ptList.ConvertAll(pt => new Point3d(pt.X, pt.Y, pt.Z));
            return newList;
        }

        public static Brep Extrude(Brep planar, Vector3d vector, double tolerance)
        {
             Brep extrusion = Extrude(planar.Curves3D, vector, tolerance);
            return extrusion;
        }

        public static Brep Extrude(IEnumerable<Curve> curves, Vector3d vector, double tolerance)
        {
            List<Brep> extrusions = new List<Brep>();

            foreach (Curve c in curves)
            {
                Brep b = Surface.CreateExtrusion(c, vector).ToBrep();
                extrusions.Add(b);
            }
            extrusions = SanCheck(extrusions);
            return Brep.JoinBreps(extrusions, tolerance)[0];
        }

        // Do not generate extrusion from curve => Potential Rhinocommon Bugs. 
        // note: curve orientation might cause some trouble...
        public static Brep Extrude(Curve curve, Vector3d vector, double tolerance)
        {
            var planar = Brep.CreatePlanarBreps(curve, tolerance)[0];
            Brep extrusion = Extrude(planar, vector, tolerance);
            return extrusion;
        }


        // ----- Union Methods -----
        public static List<Mesh> neighborMesh()
        {
            return null;
        }

        public static Double getCosSimilarity(Vector3d v1, Vector3d v2)
        {
            return Math.Abs((v1 * v2) / (v1.Length * v2.Length));
        }

        public static Vector3d normalizeVector(Vector3d v)
        {
            return v / v.Length;
        }

        // ------ Planes ---------
        public static List<Plane> SetPlaneOriginCloseTo(List<Plane> planeList, Point3d targetPoint)
        {
            List<Plane> result = new List<Plane>();
            for (int i = 0; i < planeList.Count; i++)
            {
                if (planeList[i] == null)
                    continue;

                Plane plane = SetPlaneOriginCloseTo(planeList[i], targetPoint);
                result.Add(plane);
            }
            return result;
        }

        public static Plane SetPlaneOriginCloseTo(Plane plane, Point3d targetPoint)
        {
            plane.Origin = plane.ClosestPoint(targetPoint);
            return plane;
        }

        // Legacy Builder Patch
        public static void ExtrusionInfo(Brep extrusion, out Brep ftPlanar, out Brep capPlanar, out double height)
        {
            Point3d centroid = ComputeCentroid(extrusion);

            BrepFace ftFace = null;
            BrepFace capFace = null;
            Point3d ftCentroid = centroid;
            Point3d capCentroid = centroid;

            // Analyze each face. Not the optimal solution for what we need here. 
            foreach (BrepFace bf in extrusion.Faces)
            {
                double u, v;
                bf.ClosestPoint(centroid, out u, out v);
                Point3d cloestPoint = bf.PointAt(u, v);
                if (cloestPoint.Z <= ftCentroid.Z)
                {
                    ftCentroid = cloestPoint;
                    ftFace = bf;
                }

                if (cloestPoint.Z >= capCentroid.Z)
                {
                    capCentroid = cloestPoint;
                    capFace = bf;
                }
            }
            ftPlanar = ftFace.DuplicateFace(true);
            capPlanar = capFace.DuplicateFace(true);
            height = Math.Abs(capCentroid.Z - ftCentroid.Z);
        }
    }

}
