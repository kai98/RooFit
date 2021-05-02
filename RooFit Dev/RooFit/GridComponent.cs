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
    public class GridComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public GridComponent()
          : base("RooFit Grid", "Grid",
              "Grid Sampling in XY Plane",
              "RooFit", "Dev")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // 0. Points input
            pManager.AddPointParameter("Points", "Points", "Points", GH_ParamAccess.list);

            // 1  . Density level. Default: 1.2 X density
            pManager.AddNumberParameter("Density", "Density", "Density", GH_ParamAccess.item, 1);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // Grided points. 
            pManager.AddPointParameter("Points", "Points", "Points", GH_ParamAccess.list);

            pManager.AddMeshParameter("Mesh", "Mesh", "Mesh", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>




        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> inPts = new List<Point3d>();
            double density = 1.2;

            Mesh delMesh = new Mesh();

            // input
            double tol = 0.001;

            DA.GetDataList(0, inPts);
            DA.GetData(1, ref density);


            // if density <= 0, return the original value. 
            if (density <= 0)
            {
                DA.SetDataList(0, inPts);
                return;
            }
                

            // Generate grid points 
            delMesh = DelaunayMesh2(inPts);
            Plane xyPlane = Plane.WorldXY;


            Polyline outline = delMesh.GetOutlines(xyPlane)[0];

            if (outline == null)
            {
                DA.SetDataList(0, inPts);
                return;
            }

            PolylineCurve outcurve = new PolylineCurve(outline);




            var planar = Brep.CreatePlanarBreps(outcurve, tol)[0];
            double planarArea = planar.GetArea();


            // Calculate the bounding x y value for polyline. 
            double minX = Double.PositiveInfinity;
            double minY = Double.PositiveInfinity;

            double maxX = Double.NegativeInfinity;
            double maxY = Double.NegativeInfinity;

            foreach(Point3d pt in inPts)
            {
                maxX = Math.Max(maxX, pt.X);
                maxY = Math.Max(maxY, pt.Y);
                minX = Math.Min(minX, pt.X);
                minY = Math.Min(minY, pt.Y);
            }

            double boundingArea = (maxX - minX) * (maxY - minY);

            double numInput = Convert.ToDouble(inPts.Count);

            double numOfPts = density * numInput * (boundingArea / planarArea);

            double stride = Math.Sqrt(boundingArea / numOfPts);

            // Add padding = stride / 2 to minX, minY, maxX, maxY
            maxX += stride / 2;
            maxY += stride / 2;
            minX -= stride / 2;
            minY -= stride / 2;

            // Generate grid points in XY Plane
            List<Point3d> gridPoints = new List<Point3d>();

            for(double i = minX; i < maxX; i += stride)
            {
                for(double j = minY; j < maxY; j += stride)
                {
                    Point3d thisPoint = new Point3d(i, j, 0);
                    gridPoints.Add(thisPoint);
                }
            }

            List<Mesh> mesh = new List<Mesh>();
            mesh.Add(delMesh);
            Vector3d zDirection = new Vector3d(0, 0, 1);
            List<Point3d> projectedPoints = Intersection.ProjectPointsToMeshes(mesh, gridPoints, zDirection, tol).ToList();
           
            DA.SetDataList(0, projectedPoints);
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return Properties.Resources.grid;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("5a74efd2-125f-4e53-906e-e7c74952244c"); }
        }
    }
}