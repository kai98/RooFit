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
    class Builder_v2
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

        // Patch code, Get height and footprint plannar from extrusion. 
        public Brep ftPlanar, capPlanar;
        public double height;

        // Outputs
        public Brep solid = new Brep();
        public List<Brep> selectedFragments = new List<Brep>();

        public Builder_v2(List<Point3d> _pts, Brep _extrusion, List<Plane> _planes, double _n = -1, double tol = 0.001)
        {
            pts = _pts;
            extrusion = _extrusion;
            planeList = _planes;

            // n -> number of planes will be considered. 
            n = _n;
        }

        public void Solve()
        {
            // Adapt the previous polyfit modeling code into this testing code. 
            // Still designing the better reconstruction algorithm. 

            // Set the origin of planes to the centroid of extrusion. 

            // Brep ftPlanar = Brep.CreatePlanarBreps(ftCurve, tol)[0];
            Brep ftSolid = extrusion;
            List<Plane> refinedPlanes = planeList;
            var pointCloud = pts;

            // Threshold = n..
            var threshold = n;


            // Get ftPlanr, capPlanr and height from extrusion. 
            ExtrusionInfo(extrusion, out ftPlanar, out capPlanar, out height);


            List<Brep> refinedPlanars = PlanesToPlanars(refinedPlanes, ftSolid, tol);

            List<Brep> fragments = GetBrepsFragments(refinedPlanars, ftPlanar, capPlanar, tol);

            fragments = SanCheck(fragments);
            List<Brep> brepFaces = PlanarSelection(fragments, pointCloud, threshold, tol);

            // RooFit debugging

            solid = BuildSolid(brepFaces, ftPlanar, height, tol);
            selectedFragments = brepFaces;

            // new code: merge coplanar faces. 
            // angleTolerance 15 degrees
            solid.MergeCoplanarFaces(0.1, 15);
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
            //Double length = diagonal.Length / 2 + maxHeight;
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
                    planarList.Add(planar);
            }
            // Plane Operations: Refine Extrated Planes, Generate Wall Planes.. etc. 
            return planarList;
        }


        // Ignored vertical fragments... 
        public List<Brep> GetBrepsFragments(List<Brep> refinedPlanars, Brep ftPlanar, Brep capPlanar, double tolerance)
        {
            List<Brep> fragmentList = new List<Brep>();
            foreach (Brep planar in refinedPlanars)
            {
                //if (IsVerticalPlanar(planar))
                //    continue;
                fragmentList.AddRange(GetFragments(planar, refinedPlanars));
            }

            // RooFit debug: no need to add ftFragments. 
            // ftFragments
            //List<Brep> ftFragments = GetFragments(ftPlanar, refinedPlanars);
            //fragmentList.AddRange(ftFragments);

            // cap planar
            //Vector3d vec = new Vector3d(0, 0, height);
            //Brep capPlanar = ftPlanar.DuplicateBrep();
            //capPlanar.Transform(Transform.Translation(vec));
            //List<Brep> capFragments = GetFragments(capPlanar, refinedPlanars);
            //fragmentList.AddRange(capFragments);

            fragmentList = SanCheck(fragmentList);
            return fragmentList;

            // TODO: Vertical Splits
            bool IsVerticalPlanar(Brep p)
            {
                Curve curve = Curve.JoinCurves(p.Curves3D)[0];
                Plane plane;
                curve.TryGetPlane(out plane);
                return plane.Normal.Z == 0;
            }

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

        public List<Brep> PlanarSelection(List<Brep> fragments, List<Point3d> pointCloud, double threshold, double tolerance)
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


            // if count == 0, choose the planar that should be there. Forexample, the most similar one to fit the cloest 3 points.. etc. 
            Brep GetMostInlierPlanar(List<Brep> candidateList)
            {
                int mostCount = 0;
                Brep currPlanar = candidateList[0];

                foreach (Brep candidate in candidateList)
                {
                    if (candidate == null)
                        continue;

                    int numInliers = GetNumOfInliers(candidate);
                    if (numInliers > mostCount)
                    {
                        mostCount = numInliers;
                        currPlanar = candidate;
                    }
                }

                // TODO: May 01, 2021. Fix: planar with no inlier points. 

                return currPlanar;
            }

            int GetNumOfInliers(Brep planar)
            {
                if (numDict.ContainsKey(planar))
                {
                    return numDict[planar];
                }
                else
                {
                    int count = CountInliers(planar, pointCloud);
                    numDict.Add(planar, count);
                    return count;
                }
            }

            int CountInliers(Brep planar, List<Point3d> pointList)
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
        public int CrossValidation(Brep fragmentA, Brep fragmentB, double tolerance)
        {

            // TODO: Memoization 

            Curve curveA = GetProjectedCurve(fragmentA);
            Curve curveB = GetProjectedCurve(fragmentB);

            if (curveA == null || curveB == null)
            {
                return 0;
            }

            Brep brepA = GetProjectedBrep(fragmentA, tolerance);
            Brep brepB = GetProjectedBrep(fragmentB, tolerance);

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
        public Curve GetProjectedCurve(Brep brep)
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

        public Brep GetProjectedBrep(Brep brep, double tolerance)
        {
            if (projectedBrepDict.ContainsKey(brep))
            {
                return projectedBrepDict[brep];
            }
            Curve curve = GetProjectedCurve(brep);
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
            neighborFragments = RemoveConfliction(currPlanar, neighborFragments, tol);

            // update otherFragments
            List<Brep> tempList = new List<Brep>();
            foreach (Brep fragment in otherFragments)
            {
                if (fragment == null)
                    continue;

                int relationship = CrossValidation(currPlanar, fragment, tol);
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

        public List<Brep> RemoveConfliction(Brep planar, List<Brep> brepList, double tolerance)
        {
            if (brepList == null)
                return brepList;
            brepList.RemoveAll(p => CrossValidation(planar, p, tolerance) == 0);
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
            List<Brep> extrusions = new List<Brep>();
            Curve[] nakedEdge = brep.DuplicateNakedEdgeCurves(true, false);

            foreach (Curve c in nakedEdge)
            {
                Brep b = Surface.CreateExtrusion(c, vector).ToBrep();
                extrusions.Add(b);
            }
            return Brep.JoinBreps(extrusions, tolerance)[0];
        }

        public Brep TrimExtrusion(Brep ftPlanar, Brep Extrusion, double tolerance)
        {
            Brep[] breps = Extrusion.Split(ftPlanar, tolerance);

            Point3d centroid0 = AreaMassProperties.Compute(breps[0]).Centroid;
            Point3d centroid1 = AreaMassProperties.Compute(breps[1]).Centroid;
            if (centroid0.Z > centroid1.Z)
                return breps[0];
            return breps[1];
        }

    }
}
