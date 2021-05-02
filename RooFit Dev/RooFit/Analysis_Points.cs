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
    // Random sample generated from points, plane evaluated by meshfaces. 
    class Analysis_Points
    {
        // Rhino tolerance. Default 0.001
        double tol;
        // RANSAC Data and Parameters
        // Data - Mesh
        Mesh delMesh;
        // RANSAC - Threshold value. Threshold value to be considered as inlier. 
        double threshold;
        // RANSAC - Maximum number of iterations. Adaptive.
        int numOfIter;
        // RANSAC - Probability that one meshface is belonging to one target plane. 
        double p;

        // RANSAC - d – Number of close data points required to assert that a model fits well to data.
        int d;

        // Data from delMesh. Should be constant. 
        readonly List<Point3f> vertices;
        readonly List<MeshFace> faces;

        // rd for random
        Random rd;

        // Ransac result
        public List<HashSet<MeshFace>> extractedFaces = new List<HashSet<MeshFace>>();

        // Results
        // Mesh List  
        public List<Mesh> resultMeshList = new List<Mesh>();

        // Plane List
        public List<Plane> resultPlaneList = new List<Plane>();

        // isRefit?
        bool isRefit = false;
        int maxCount = 20;


        public Analysis_Points(Mesh _delMesh, double _threshold = 0.25, double _tol = 0.001)
        {
            this.delMesh = _delMesh;
            this.threshold = _threshold;
            this.tol = _tol;
            this.p = 0.1;
            this.d = 4;

            // TODO: Adaptive estimation. 

            // A coarse estimation for the first plane. 
            // if p is 0.1 for the first plane, then 1 - (1-0.1) ** 50 = 0.9948. 
            // Will have 99.48 % of chance to find the target plane.
            // Note: numOfIter will be updated according to the p from previous round. 
            this.numOfIter = 1000;

            this.vertices = new List<Point3f>(_delMesh.Vertices);
            this.faces = new List<MeshFace>(_delMesh.Faces);

            this.rd = new Random(42);

        }

        public void Solve()
        {
            isRefit = true;
            if (isRefit)
                maxCount = 5;

            RANSAC_FindPlanes(vertices, faces);
            GenerateResults();
        }

        public void GenerateResults()
        {
            foreach (HashSet<MeshFace> mfSet in extractedFaces)
            {

                // Fit a plane to a set of mesh according to vertex points. 
                Mesh thisMesh = BuildMesh(mfSet);
                Plane thisPlane = FitPlane(mfSet);

                resultMeshList.Add(thisMesh);
                resultPlaneList.Add(thisPlane);
            }

            // Set the plane's origin close to mesh centroid
            Point3d centroid = ComputeCentroid(delMesh);
            resultPlaneList = SetPlaneOriginCloseTo(resultPlaneList, centroid);
        }

        public Plane FitPlane(IEnumerable<MeshFace> meshFaces)
        {
            HashSet<int> pts = GetVertexSet(meshFaces);
            return FitPlane(pts);
        }

        public HashSet<int> GetVertexSet(IEnumerable<MeshFace> meshFaces)
        {
            HashSet<int> pts = new HashSet<int>();
            foreach (MeshFace mf in meshFaces)
            {
                pts.Add(mf.A);
                pts.Add(mf.B);
                pts.Add(mf.C);
                pts.Add(mf.D);
            }
            return pts;
        }

        public Mesh BuildMesh(IEnumerable<MeshFace> meshFaces)
        {
            Mesh m = new Mesh();
            m.Vertices.AddVertices(vertices);
            m.Faces.AddFaces(meshFaces);
            m.Compact();
            return m;
        }

        public void RANSAC_FindPlanes(List<Point3f> pts, List<MeshFace> faces)
        {
            // candidate list for plane guessing. 
            HashSet<MeshFace> candidates = new HashSet<MeshFace>(faces);

            int iter = numOfIter;

            // Fail safe. Upper bound. Should be the estimated number of planes. 20 will be a good option. 
            // int maxCount = faces.Count;
            maxCount = 50;

            for (int i = 0; i < maxCount; i++)
            {
                // the size of candidates set should be reduced in every loop. 
                if (candidates.Count <= 0)
                    break;

                // Get inliers from candidates. The size of set should be greater than zero. 
                HashSet<MeshFace> inliers = RansacPlane(candidates, iter);

                // TODO: use probability to update the max iter for next RansacPlane. 

                // Inliers and Extracted planes will be added to the global variables inside RansacPlane method. 
                // Update candidates. 
                candidates.ExceptWith(inliers);

                // Early stop. If the best plane has inliers less than d.
                // d – Number of close data points required to assert that a model fits well to data.
                // Then stop the loop. 
                //if (inliers.Count < this.d)
                //    break;
            }
        }

        public HashSet<MeshFace> RansacPlane(HashSet<MeshFace> candidates, int iter)
        {
            // if there is no available candidate, return an empty set. 
            if (candidates.Count <= 0)
                return new HashSet<MeshFace>();

            if (candidates.Count < iter)
                iter = candidates.Count;

            int bestCount = 0;

            // Plane bestPlane = new Plane();

            HashSet<MeshFace> inliers = new HashSet<MeshFace>();


            for (int i = 0; i < iter; i++)
            {
                if (candidates.Count < 0)
                    break;

                Plane rdPlane = GetRandomPlaneFromVertex(candidates);

                HashSet<MeshFace> thisInliers = GetInlierMeshFaces(rdPlane, candidates);

                if (isRefit)
                {
                    // refit with inliers, 04/27
                    // If refit is true, set the maxIter to 5 or 10. we need early stop. 
                    Plane refitPlane = FitPlane(thisInliers);
                    HashSet<MeshFace> refitInliers = GetInlierMeshFaces(refitPlane, candidates);
                    thisInliers = refitInliers;
                }

                if (bestCount < thisInliers.Count)
                {
                    // bestPlane = rdPlane;
                    inliers = thisInliers;
                    bestCount = thisInliers.Count;
                }
            }

            // make sure the bestMF is also in the inlier set...
            // inliers.Add(bestMF);



            // update global variables
            if (bestCount > this.d)
            {
                extractedFaces.Add(inliers);
            }

            return inliers;
        }

        // Get RandomPlane from list of meshface
        public Plane GetRandomPlaneFromFace(HashSet<MeshFace> mfList)
        {
            MeshFace rdMF = GetRandomFace(mfList);
            Plane rdPlane = MF2Plane(rdMF);
            return rdPlane;
        }

        public Plane GetRandomPlaneFromVertex(HashSet<MeshFace> mfList)
        {

            //HashSet<int> random3Points = new HashSet<int>();
            HashSet<Point3d> random3Points = new HashSet<Point3d>();

            while (random3Points.Count < 3)
            {
                MeshFace mf = GetRandomFace(mfList);
                int ptindex = 0;
                int i = rd.Next(3);
                if (i == 0)
                    ptindex = mf.A;
                if (i == 1)
                    ptindex = mf.B;
                if (i == 2)
                    ptindex = mf.C;
                random3Points.Add(vertices[ptindex]);
            }

            List<Point3d> ptList = new List<Point3d>(random3Points);
            Plane p = new Plane(ptList[0], ptList[1], ptList[2]);
            return p;
        }

        // TODO: Rewrite
        public HashSet<MeshFace> GetInlierMeshFaces(Plane p, HashSet<MeshFace> candidates)
        {
            // creates two set. One for visited vertices, one for valid vertices. 
            Dictionary<int, bool> inlierDictionary = new Dictionary<int, bool>();

            HashSet<MeshFace> inliers = new HashSet<MeshFace>();

            foreach (MeshFace mf in candidates)
            {
                if (isInlierMF(p, mf, inlierDictionary))
                    inliers.Add(mf);
            }

            return inliers;
        }

        // Like a Bloom Filter
        public bool isInlierMF(Plane p, MeshFace mf, Dictionary<int, bool> inlierDictionary)
        {
            List<int> pts = new List<int>()
            {
                mf.A,
                mf.B,
                mf.C,
                // mf.D
            };

            foreach (int i in pts)
            {
                // the point is not an inlier point. Return false;
                if (!isInlierPt(p, i, inlierDictionary))
                    return false;
            }
            return true;
        }

        public Plane FitPlane(IEnumerable<int> validIndices)
        {
            HashSet<Point3d> validPts = new HashSet<Point3d>();
            foreach (int index in validIndices)
                validPts.Add(vertices[index]);

            // Error...
            if (validPts.Count < 3)
                return new Plane();

            Plane plane;
            double maxDev;
            Plane.FitPlaneToPoints(validPts, out plane, out maxDev);

            // Maybe we can update the threshold with maxDev. 
            // TODO: adaptive threshold implementation after the initial guess

            return plane;
        }


        public bool isInlierPt(Plane p, int index, Dictionary<int, bool> inlierDictionary)
        {
            if (inlierDictionary.ContainsKey(index))
                return inlierDictionary[index];

            Point3f pt = vertices[index];
            double absDiff = Math.Abs(p.DistanceTo(pt));
            bool flag = absDiff <= threshold;
            inlierDictionary[index] = flag;
            return flag;
        }

        public MeshFace GetRandomFace(HashSet<MeshFace> candidates)
        {
            // rd for random. 
            int mfi = rd.Next(candidates.Count);
            MeshFace rdMF = candidates.ElementAt(mfi);
            return rdMF;
        }

        // Get a plane from a meshface
        public Plane MF2Plane(MeshFace mf)
        {
            return new Plane(vertices[mf.A], vertices[mf.B], vertices[mf.C]);
        }

    }
}
