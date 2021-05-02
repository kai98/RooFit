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
    class Distributor
    {
        // inputs
        List<Point3d> pts = new List<Point3d>();
        List<Plane> planeList = new List<Plane>();
        int n = 5; // top n plane. 
        Mesh delMesh = new Mesh();

        // outputs
        List<Plane> topPlanes = new List<Plane>();

        // Global Parameters
        double tol = 0.001;


        // Readonly Datab
        readonly List<Point3f> vertices;
        readonly List<MeshFace> faces;


        // Closest point count
        Dictionary<Plane, int> countDict = new Dictionary<Plane, int>();

        // Distance Dictionary
        Dictionary<Point3d, Dictionary<Plane, double>> pointDistanceMap = new Dictionary<Point3d, Dictionary<Plane, double>>();
        Dictionary<MeshFace, Dictionary<Plane, double>> meshfaceDistanceMap = new Dictionary<MeshFace, Dictionary<Plane, double>>();

        // Meshface set to the cloest plane
        Dictionary<Plane, HashSet<MeshFace>> planeMeshFaces = new Dictionary<Plane, HashSet<MeshFace>>();

        public Distributor(List<Point3d> _pts, Mesh _delMesh, List<Plane> _planeList, int _n=5)
        {
            this.pts = _pts;
            this.delMesh = _delMesh;
            this.planeList = _planeList;
            this.n = _n;
            this.vertices = new List<Point3f>(_delMesh.Vertices);
            this.faces = new List<MeshFace>(_delMesh.Faces);
        }

        public List<Mesh> Solve()
        {
            AppendMFtoCloestPlane(this.faces, this.planeList);
            List<Mesh> result = GeneratePlaneMeshes();
            return result;
        }


        void AppendMFtoCloestPlane(List<MeshFace> mfList, List<Plane> planeList)
        {
            foreach(MeshFace mf in mfList)
            {
                double minDistance = Double.MaxValue;
                Plane cloestPlane = planeList[0];

                foreach (Plane plane in planeList)
                {
                    double currentDistance = Face2Plane(mf, plane);

                    if (currentDistance < minDistance)
                    {
                        minDistance = currentDistance;
                        cloestPlane = plane;
                    }
                }

                // Add the current meshface to its cloest plane
                if (!planeMeshFaces.ContainsKey(cloestPlane))
                    planeMeshFaces[cloestPlane] = new HashSet<MeshFace>();
                planeMeshFaces[cloestPlane].Add(mf);
            }
        }

        List<Mesh> GeneratePlaneMeshes()
        {
            List<Mesh> result = new List<Mesh>();
            foreach(Plane plane in this.planeList)
            {
                List<MeshFace> mflist = new List<MeshFace>(planeMeshFaces[plane]);
                Mesh currentMesh = new Mesh();
                currentMesh.Faces.AddFaces(mflist);
                currentMesh.Vertices.AddVertices(vertices);
                currentMesh.Compact();
                result.Add(currentMesh);
            }
            return result;
        }

        void CountCloestPoints(List<Point3d> pts, List<Plane> planeList)
        {
            // initialize the dictionary
            foreach(Plane p in planeList)
            {
                countDict[p] = 0;
            }


            foreach(Point3d pt in pts)
            {
                double minDistance = Double.MaxValue;
                Plane minPlane = planeList[0];
                foreach(Plane p in planeList)
                {
                    double currentDistance = Point2Plane(pt, p);

                    if (currentDistance < minDistance)
                    {
                        minDistance = currentDistance;
                        minPlane = p;
                    }
                }

                // count ++
                countDict[minPlane] += 1;

            }
        }

        // Distance from meshface to points. With memoization. 
        public double Face2Plane(MeshFace mf, Plane plane)
        {
            if (!meshfaceDistanceMap.ContainsKey(mf))
            {
                meshfaceDistanceMap[mf] = new Dictionary<Plane, double>();
            }

            var distances = meshfaceDistanceMap[mf];

            if (!distances.ContainsKey(plane)) {
                double totalDistance = 0;
                totalDistance += Point2Plane(vertices[mf.A], plane);
                totalDistance += Point2Plane(vertices[mf.B], plane);
                totalDistance += Point2Plane(vertices[mf.C], plane);

                distances[plane] = totalDistance;
            }
            return distances[plane];
        }


        // Using memoization, store the distance from plane to point. 
        public double Point2Plane(Point3d pt, Plane plane)
        {
            // if distance map doesn't have the current point as key, initialize a new value. 
            if (!pointDistanceMap.ContainsKey(pt))
                pointDistanceMap[pt] = new Dictionary<Plane, double>();

            // pointDistances => Dictionary<Plane, double>; 
            var distances = pointDistanceMap[pt];

            if (!distances.ContainsKey(plane))
                distances[plane] = Math.Abs(plane.DistanceTo(pt));

            return distances[plane];
        }

    }
}
