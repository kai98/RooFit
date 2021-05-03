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
    class Builder
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

        // inlier threshold. (Max Dev from previous step)
        double threshold = 0;

        // Parameter to control the reconstruction performance if applicable.
        int n = 4;
        bool sortTopN;

        // Data
        // Plane -> Set of Inlier Points 
        Dictionary<Plane, HashSet<Point3d>> planeInliers = new Dictionary<Plane, HashSet<Point3d>>();

        // Patch code, Get height and footprint plannar from extrusion. 
        public Brep ftPlanar, capPlanar;
        public double height;

        // Outputs
        public Brep solid = new Brep();
        public List<Brep> selectedFragments = new List<Brep>();

        HashSet<Point3d> totalInliers = new HashSet<Point3d>();


        public Builder(List<Point3d> _pts, Brep _extrusion, List<Plane> _planes, double _threshold, int _n=4, double tol=0.001)
        {
            this.pts = _pts;
            this.extrusion = _extrusion;
            this.planeList = _planes;
            this.threshold = _threshold;
            this.n = _n; // n -> number of planes will be considered. 

            // Do not sort the planes.. 
            sortTopN = false;
        }

        public void Solve()
        {
            // Adapt the previous polyfit modeling code into this testing code. 
            // Still designing the better reconstruction algorithm. 

            // Set the origin of planes to the centroid of extrusion. 

            // Brep ftPlanar = Brep.CreatePlanarBreps(ftCurve, tol)[0];
            Brep ftSolid = extrusion;
            List<Plane> refinedPlanes = planeList;
            var points = pts;


            // Get ftPlanr, capPlanr and height from extrusion. 
            ExtrusionInfo(extrusion, out ftPlanar, out capPlanar, out height);

            // Vertical planes might produce unexpected outcput in the final output. 
            // TODO: Vertical cuts can solve the problem in the next version. 
            // Hardcoded to 0.05
            double z_value = 0.05;
            refinedPlanes = ExceptVerticalPlanes(refinedPlanes, z_value);

            // Select the top N planes. 
            refinedPlanes = TopNPlanes(refinedPlanes, this.n);

            // Speed-up, May 03;
            // Get inliers for each plane. 
            var planeInlierMap = GetPlaneInlierSets(refinedPlanes, points, threshold);

            List<Brep> refinedPlanars = PlanesToPlanars(refinedPlanes, ftSolid, tol);

            List<Brep> fragments = GetBrepsFragments(refinedPlanars, tol);

            fragments = SanCheck(fragments);
            List<Brep> brepFaces = PlanarSelection(fragments, threshold, tol);

            // RooFit debugging

            solid = BuildSolid(brepFaces, ftPlanar, height, tol);
            selectedFragments = brepFaces;

            // new code: merge coplanar faces. 
            // angleTolerance 20 degrees
            solid.MergeCoplanarFaces(0.5, 20);
        }

        // Getting the top n planes from plane inliers. 
        List<Plane> TopNPlanes(List<Plane> planeList, int n)
        {
            var sortedList = new List<Plane>(planeList);

            // sort the list. From largest to smallest, so take negative value. 
            if (sortTopN)
            {
                sortedList.Sort(delegate (Plane a, Plane b)
                {
                    return -(planeInliers[a].Count - planeInliers[b].Count);
                });
            }
            return new List<Plane>(sortedList.Take(n));
        }


        // Comparer for Brep... based on the number of inliers. 

        Dictionary<Plane, HashSet<Point3d>> GetPlaneInlierSets(List<Plane> planeList, List<Point3d> points, double threshold)
        {
            var inlierMap = new Dictionary<Plane, HashSet<Point3d>>();
            foreach(Plane plane in planeList)
            {
                var inlierSet = GetInliers(plane, points, threshold);
                inlierMap[plane] = inlierSet;

                // May 03 Speed-up
                this.totalInliers.UnionWith(inlierSet);
            }
            return inlierMap;
        }

        HashSet<Point3d> GetInliers(Plane plane, List<Point3d> points, double threshold)
        {
            HashSet<Point3d> inlierSet = new HashSet<Point3d>();
            foreach(Point3d pt in points)
            {
                double absDiff = Math.Abs(plane.DistanceTo(pt));
                if (absDiff <= threshold)
                    inlierSet.Add(pt);
            }
            return inlierSet;
        }

        // Can be used to sepearte vertical planes and use them to seperate different smaller pieces in the next version. 
        List<Plane> ExceptVerticalPlanes(List<Plane> planeList, double z_value)
        {
            List<Plane> result = new List<Plane>();
            foreach(Plane plane in planeList)
            {
                if (Math.Abs(plane.Normal.Z) > z_value)
                    result.Add(plane);
            }
            return result;
        }

        // Methods from PolyFitv3 SolidBuilder 
        // Planes to Planars... integrated method

        // Planes to Planars... integrated method
        // RooFit: need to set planes' origin to extrusion's origin
        public List<Brep> PlanesToPlanars(List<Plane> refinedPlanes, Brep ftExtrusion, double tolerance)
        {
            Vector3d diagonal = ftExtrusion.GetBoundingBox(false).Diagonal;

            // RooFit compatible. 
            Point3d centroid = ComputeCentroid(ftExtrusion);
            List<Plane> planeList = SetPlaneOriginCloseTo(refinedPlanes, centroid);

            double length = diagonal.Length;
            Interval maxInterval = new Interval(-length, length);
            List<PlaneSurface> surfaceList = new List<PlaneSurface>();

            List<Brep> planarList = new List<Brep>();

            for (int i = 0; i < planeList.Count; i++)
            {
                Plane plane = planeList[i];

                // RooFit compatible, set origin
                // plane.Origin = centroid;

                PlaneSurface pSurface = new PlaneSurface(plane, maxInterval, maxInterval);
                surfaceList.Add(pSurface);

                Brep brep = pSurface.ToBrep();
                Brep planar = brep.Trim(ftExtrusion, tolerance)[0];
                if (planar != null)
                {
                    planarList.Add(planar);
                }
            }
            // Plane Operations: Refine Extrated Planes, Generate Wall Planes.. etc. 
            return planarList;
        }


        // Ignored vertical fragments... 
        //public List<Brep> GetBrepsFragments(List<Brep> refinedPlanars, Brep ftPlanar, Brep capPlanar, double tolerance)

        public List<Brep> GetBrepsFragments(List<Brep> planarList, double tolerance)
        {
            List<Brep> fragmentList = new List<Brep>();
            foreach (Brep currentPlanar in planarList)
            {
                List<Brep> currentFragments = GetFragments(currentPlanar, planarList);
                fragmentList.AddRange(currentFragments);
            }

            fragmentList = SanCheck(fragmentList);
            return fragmentList;


            List<Brep> GetFragments(Brep brep, List<Brep> cutterBreps)
            {
                List<Curve> curves = new List<Curve>();
                Brep brepA = brep.DuplicateBrep();

                foreach (Brep brepB in cutterBreps)
                {
                    Curve[] intersectionCurves;
                    Point3d[] intersectionPoints;
                    Intersection.BrepBrep(brepA, brepB, tolerance, out intersectionCurves, out intersectionPoints);

                    // TODO AddRange + SanCheck
                    foreach (Curve c in intersectionCurves)
                    {
                        if (c != null)
                            curves.Add(c);
                    }
                }
                Brep[] breps = brepA.Split(curves, tolerance);
                if (breps.Length == 0)
                {
                    // keep brepA unchanged. 
                    List<Brep> tempList = new List<Brep>();
                    tempList.Add(brepA);
                    return tempList;
                }
                return breps.ToList();
            }
        }

        public List<Brep> PlanarSelection(List<Brep> fragments, double threshold, double tolerance)
        {
            Dictionary<Brep, int> numDict = new Dictionary<Brep, int>();

            // Should Sancheck fragments first. 
            fragments = SanCheck(fragments);

            List<Brep> selectedFragments = new List<Brep>();
            List<Brep> neighborFragments = new List<Brep>();
            List<Brep> otherFragments = new List<Brep>();
            otherFragments.AddRange(fragments);

            Brep firstPlanar = GetMostInlierPlanar(fragments);

            UpdateLists(firstPlanar, ref selectedFragments, ref neighborFragments, ref otherFragments, tolerance);

            while (neighborFragments != null && neighborFragments.Count != 0)
            {
                Brep currPlanar = GetMostInlierPlanar(neighborFragments);
                UpdateLists(currPlanar, ref selectedFragments, ref neighborFragments, ref otherFragments, tolerance);
            }

            return selectedFragments;

            Brep GetMostInlierPlanar(List<Brep> fragmentList)
            {
                int mostCount = 0;
                Brep currentFragment = fragmentList[0];

                foreach (Brep f in fragmentList)
                {
                    if (f == null)
                        continue;

                    int numInliers = GetNumOfInliers(f);
                    if (numInliers > mostCount)
                    {
                        mostCount = numInliers;
                        currentFragment = f;
                    }
                }
                return currentFragment;
            }

            int GetNumOfInliers(Brep fragment)
            {
                if (numDict.ContainsKey(fragment))
                {
                    return numDict[fragment];
                }
                else
                {
                    int count = CountInliers(fragment, this.totalInliers);
                    numDict.Add(fragment, count);
                    return count;
                }
            }

            int CountInliers(Brep planar, IEnumerable<Point3d> pointList)
            {
                int count = 0;
                foreach (Point3d pt in pointList)
                {
                    double distance = BrepDistanceToPoint(planar, pt);

                    // After remap the points. should be less than tol. 
                    if (distance <= threshold)
                    {
                        count++;
                    }
                }
                return count;

                double BrepDistanceToPoint(Brep brep, Point3d point)
                {
                    Point3d cloestPoint = brep.ClosestPoint(point);
                    return cloestPoint.DistanceTo(point);
                }
            }

        }

        // vertical relationship between two fragments. 0 -> conflict, 1 -> neighbor, 2-> waitlisted
        public int ProjectionRelationaship(Brep fragmentA, Brep fragmentB, double tolerance)
        {

            // TODO: Memoization 

            Curve curveA = GetCurveProjection(fragmentA);
            Curve curveB = GetCurveProjection(fragmentB);

            if (curveA == null || curveB == null)
            {
                return 0;
            }

            Brep brepA = GetBrepProjection(fragmentA, tolerance);
            Brep brepB = GetBrepProjection(fragmentB, tolerance);

            if (brepA == null || brepB == null)
            {
                return 0;
            }

            Curve[] overlapCurves;
            Point3d[] intersectionPoints;
            var successA = Intersection.CurveBrep(curveA, brepB, tolerance, out overlapCurves, out intersectionPoints);
            var overlapCurvesA = overlapCurves;

            if (overlapCurvesA == null)
            {
                return 0;
            }

            int lengthA = overlapCurvesA.Length;

            // conflict
            if (lengthA > 1)
            {
                return 0;
            }

            var successB = Intersection.CurveBrep(curveB, brepA, tolerance, out overlapCurves, out intersectionPoints);
            var overlapCurvesB = overlapCurves;

            if (overlapCurvesB == null)
            {
                return 0;
            }

            int lengthB = overlapCurvesB.Length;

            if (lengthB > 1)
            {
                return 0;
            }

            // neighbor fragment
            if (lengthA == 1 && lengthB == 1)
            {
                if (overlapCurvesA[0].IsLinear() && overlapCurvesB[0].IsLinear())
                    return 1;
                else
                    return 0;
            }

            // waitlisted
            if (lengthA == 0 && lengthB == 0)
                return 2;
            return 0;
        }

        Plane worldXY = Plane.WorldXY;
        Dictionary<Brep, Curve> projectedCurveDict = new Dictionary<Brep, Curve>();
        Dictionary<Brep, Brep> projectedBrepDict = new Dictionary<Brep, Brep>();
        public Curve GetCurveProjection(Brep brep)
        {
            if (projectedCurveDict.ContainsKey(brep))
            {
                return projectedCurveDict[brep];
            }

            var curve = Curve.JoinCurves(brep.Curves3D)[0];
            curve = Curve.ProjectToPlane(curve, worldXY);
            projectedCurveDict[brep] = curve;
            return curve;
        }

        public Brep GetBrepProjection(Brep brep, double tolerance)
        {
            if (projectedBrepDict.ContainsKey(brep))
            {
                return projectedBrepDict[brep];
            }
            Curve curve = GetCurveProjection(brep);
            Brep newBrep = Brep.CreatePlanarBreps(curve, tolerance)[0];
            projectedBrepDict[brep] = newBrep;
            return newBrep;
        }

        public void UpdateLists(Brep currPlanar, ref List<Brep> selectedFragments, ref List<Brep> neighborFragments, ref List<Brep> otherFragments, double tolerance)
        {
            // remove currPlanar from list. 
            neighborFragments.Remove(currPlanar);
            otherFragments.Remove(currPlanar);

            // add to selectedFragments
            selectedFragments.Add(currPlanar);

            // update neighborFragments
            neighborFragments = RemoveConflictFragments(currPlanar, neighborFragments, tol);

            // update otherFragments
            List<Brep> tempList = new List<Brep>();
            foreach (Brep fragment in otherFragments)
            {
                if (fragment == null)
                    continue;

                int relationship = ProjectionRelationaship(currPlanar, fragment, tol);
                if (relationship == 1)
                {
                    neighborFragments.Add(fragment);
                }
                else if (relationship == 2)
                {
                    tempList.Add(fragment);
                }
            }
            otherFragments = tempList;
        }

        public List<Brep> RemoveConflictFragments(Brep planar, List<Brep> brepList, double tolerance)
        {
            if (brepList == null)
                return brepList;
            brepList.RemoveAll(p => ProjectionRelationaship(planar, p, tolerance) == 0);
            return brepList;
        }

        public Brep BuildSolid(List<Brep> brepFaces, Brep ftPlanar, double height, double tolerance)
        {
            Brep roofBrep = Brep.JoinBreps(brepFaces, tolerance)[0];
            Brep downExtrusion = GetExtrusion(roofBrep, new Vector3d(0, 0, -1.5 * height), tolerance);
            roofBrep.Join(downExtrusion, tolerance, false);
            roofBrep = TrimExtrusion(ftPlanar, roofBrep, tolerance);

            Brep solid = roofBrep.CapPlanarHoles(tolerance);
            return solid;
        }


        public Brep GetExtrusion(Brep brep, Vector3d vector, double tolerance)
        {
            // Curve curve = Curve.JoinCurves(planar.Curves3D, tolerance)[0];
            List<Brep> extrudes = new List<Brep>();
            Curve[] nakedEdge = brep.DuplicateNakedEdgeCurves(true, false);

            foreach (Curve c in nakedEdge)
            {
                Brep b = Surface.CreateExtrusion(c, vector).ToBrep();
                extrudes.Add(b);
            }
            return Brep.JoinBreps(extrudes, tolerance)[0];
        }


        // either trimextrusion is OK. Just make sure we are getting the right one. 

        public Brep TrimExtrusion(Brep ftPlanar, Brep Extrusion, double tolerance)
        {
            Brep[] breps = Extrusion.Split(ftPlanar, tolerance);

            Point3d centroid0 = AreaMassProperties.Compute(breps[0]).Centroid;
            Point3d centroid1 = AreaMassProperties.Compute(breps[1]).Centroid;
            if (centroid0.Z > centroid1.Z)
                return breps[0];
            return breps[1];
        }

        // Assumed that ftPlanar is pointing downward. Therefore the main part of building was kept. 
        //public Brep TrimExtrusion(Brep ftPlanar, Brep roofExtrude, double tol)
        //{
        //    // Using Brep.Trim Method. 
        //    // Note: the parts of the brep that lie inside of the cutter are retained while the parts to the outside are discarded.
        //    Brep[] brepsInside = roofExtrude.Trim(ftPlanar, tol);
        //    Brep trimedExtrude = Brep.JoinBreps(brepsInside, tol)[0];
        //    return trimedExtrude;
        //}
    }
}
