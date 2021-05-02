using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace RooFit
{
    public class DistributorComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public DistributorComponent()
          : base("Distribution", "Distribution",
              "Distribution in Builder",
              "RooFit", "Builder Dev")
        {  
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "Points", "Points", GH_ParamAccess.list);

            pManager.AddMeshParameter("Mesh", "Mesh", "Mesh", GH_ParamAccess.item);

            pManager.AddPlaneParameter("Planes", "Planes", "Planes", GH_ParamAccess.list);

            pManager.AddIntegerParameter("n", "n", "n", GH_ParamAccess.item); 
        }
        
        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Meshes", "Meshes", "Meshes", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> pts = new List<Point3d>();
            Mesh delMesh = new Mesh();
            List<Plane> planes = new List<Plane>();
            int n = 0;
            DA.GetDataList(0, pts);
            DA.GetData(1, ref delMesh);
            DA.GetDataList(2, planes);
            DA.GetData(3, ref n);


            Distributor dt = new Distributor(pts, delMesh, planes, n);
            List<Mesh> meshes = dt.Solve();
            DA.SetDataList(0, meshes);
            // DA.SetDataList(1, lb.selectedFragments);
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
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("a4458f4b-6030-4da4-9124-002a1bdddfe1"); }
        }
    }
}