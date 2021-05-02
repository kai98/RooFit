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
    class Refinement
    {
        // inputs
        Brep extrusion;
        List<Plane> planeList;
        double xyThreshold;
        double zThreshold;

        // data
        List<Vector3d> extrusion_normals;
        List<Plane> extrusion_planes; // or wall planes
        //List<Vector3d> extrusion_XY;
        // List<Vector3d> extrusion_Z;

        // outputs
        public List<Plane> refinedPlanes;

        public Refinement(Brep _extrusion, List<Plane> _planes, double _xySimilarity, double _zSimilarity)
        {
            extrusion = _extrusion;
            planeList = _planes;
            xyThreshold = _xySimilarity;
            zThreshold = _zSimilarity;
        }

        public void Solve()
        {
            // Step 1: Generate extrusion normal and extrusion planes. Similar to extrusion_info function. 
            extrusion_normals = GetBrepVectorList(extrusion);

            // Step 2: For each plane, find the most similar xy - similarity. If less then threshold, change the xy value accordingly. (Keep the length of x y direction the same). 
            refinedPlanes = RefineWithCosSimilarity(planeList, extrusion_normals, xyThreshold);

            // Step 3: Maybe do similar thing to Z. Maybe not. Set a threshold directly for z. for example, if z < 0.1, z = 0; normalize vector. 

            // Step 4: [polyfit plan_refinement] Using wall planes to replace similar plane. 
        }

        public List<Plane> RefineWithCosSimilarity(List<Plane> pList, List<Vector3d> vectors, double threshold)
        {
            // make sure the origin of plane is correct. 
            List<Plane> results = new List<Plane>();

            for(int i = 0; i < pList.Count; i++)
            {
                Plane thisPlane = pList[i];
                // refine one plane at a time. 
                bool isSafeDistance = true;
                thisPlane = RefineWithCosSimilarity(thisPlane, vectors, threshold, ref isSafeDistance);
                if(isSafeDistance)
                    results.Add(thisPlane);
            }

            return results;
        }

        public Plane RefineWithCosSimilarity(Plane currPlane, List<Vector3d> vectors, double threshold, ref bool safeDistance)
        {
            Vector3d planeNormal = currPlane.Normal;
            double bestSimilarity = 0;
            Vector3d bestVector = planeNormal;

            foreach(Vector3d v in vectors)
            {
                double thisSimilarity = getCosSimilarity(v, planeNormal);
                if (thisSimilarity > bestSimilarity)
                {
                    bestSimilarity = thisSimilarity;
                    bestVector = v;
                }
            }
            safeDistance = true;
            if (bestSimilarity > threshold)
            {
                Plane refinedPlane = new Plane(currPlane.Origin, bestVector);

                // Also check if it is too close to Brep. 
                // A fragment with no points will cause error in legacy builder. 
                safeDistance = WithInBoundary(currPlane.Origin, extrusion, 1);
                if (safeDistance)
                {
                    return refinedPlane;
                }
                return currPlane;
            }
            else
            {
                return currPlane;
            }
        }

        public bool WithInBoundary(Point3d pt, Brep extrusion, double distance)
        {
            return extrusion.IsPointInside(pt, distance, true);
        }

        //public List<Plane> RefineXY(List<Plane> pList, List<Vector3d> vectors, double threshold)
        //{
        //    // make sure the origin of plane is correct. 
        //    List<Plane> results = new List<Plane>();
        //    List<Vector3d> vectors_XY = new List<Vector3d>();

        //    foreach (Vector3d v in vectors)
        //    {
        //        vectors_XY.Add(XYDirection(v));
        //    }

        //    for (int i = 0; i < pList.Count; i++)
        //    {
        //        Plane thisPlane = pList[i];
        //        // refine one plane at a time. 
        //        thisPlane = RefineXY(thisPlane, vectors_XY, threshold);
        //        results.Add(thisPlane);
        //    }

        //    return results;
        //}

        //public Plane RefineXY(Plane plane, List<Vector3d> vectors_XY, double threshold)
        //{
        //    Vector3d normal = XYDirection(plane.Normal);
        //    double bestSimilarity = 0;
        //    Vector3d bestVector = normal;

        //    // find the most similar vector among all extrusion face normals. 
        //    foreach(Vector3d v in vectors_XY)
        //    {
        //        double similarity = getCosSimilarity(v, normal);
        //        if (similarity > bestSimilarity)
        //        {
        //            bestSimilarity = similarity;
        //            bestVector = v;
        //        }
        //    }

        //    // successfully refined. 
        //    if (bestSimilarity >= threshold)
        //    {
        //        Vector3d refined_XY = new Vector3d(bestVector.X, bestVector.Y, 0);
        //        // normalized

        //        refined_XY /= refined_XY.Length;
        //        refined_XY *= (1 - plane.Normal.Z * plane.Normal.Z);
        //        normal = refined_XY;
        //        normal.Z = plane.Normal.Z;

        //        Point3d origin = plane.Origin;
        //        Plane refinedPlane = new Plane(origin, normal);
        //        return refinedPlane;
        //    }

        //    // no need to change. 
        //    return plane;
        //}

        public Vector3d XYDirection(Vector3d v)
        {
            return new Vector3d(v.X, v.Y, 0);
        }

        public Vector3d ZDirection(Vector3d v)
        {
            return new Vector3d(0, 0, v.Z);
        }

        public List<Vector3d> GetBrepVectorList(Brep brep)
        {
            List<Vector3d> vectorList = new List<Vector3d>();
            foreach(BrepFace bf in brep.Faces)
            {
                Vector3d normal = GetBrepFaceNormal(bf);
                vectorList.Add(normal);
            }
            return vectorList;
        }

        public Vector3d GetBrepFaceNormal(BrepFace bf)
        {
            Point3d point0 = new Point3d(0, 0, 0);
            double u, v;
            bf.ClosestPoint(point0, out u, out v);
            Vector3d normal = bf.NormalAt(u, v);
            return normal;
        }
    }

}
