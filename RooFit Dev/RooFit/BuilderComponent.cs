using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace RooFit
{
    public class BuilderComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public BuilderComponent()
          : base("Builder", "Builder",
              "Builder",
              "RooFit", "RooFit")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // 0 - Points for evaluation
            pManager.AddPointParameter("Points", "Points", "Points", GH_ParamAccess.list);

            // 1 - Extrusion
            pManager.AddBrepParameter("Extrusion", "Extrusion", "Extrusion", GH_ParamAccess.item);

            // 2 - Candidate Planes
            pManager.AddPlaneParameter("Planes", "Planes", "Planes", GH_ParamAccess.list);

            // 3 - Max Dev
            pManager.AddNumberParameter("Threshold", "Threshold", "Threshold", GH_ParamAccess.item, 5);

            // 4 - Top n planes will be considered. 
            pManager.AddIntegerParameter("Top n", "Top n", "Top n", GH_ParamAccess.item, 4);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // Final output model 
            pManager.AddBrepParameter("Solid", "Solid", "Solid", GH_ParamAccess.item);

            // Fragment Brep selected to build the model. For dev testing. 
            // pManager.AddBrepParameter("Fragments [Dev]", "Fragments [Dev]", "Fragments [Dev]", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Currently equivalent to Legacy Bulider. 
            List<Point3d> pts = new List<Point3d>();
            Brep extrusion = null;
            List<Plane> planes = new List<Plane>();
            double threshold = 0;
            int n = 0;
            DA.GetDataList(0, pts);
            DA.GetData(1, ref extrusion);
            DA.GetDataList(2, planes);
            DA.GetData(3, ref threshold);
            DA.GetData(4, ref n);

            double tol = 0.001;

            Builder b2 = new Builder(pts, extrusion, planes, threshold, n, tol);
            b2.Solve();
            DA.SetData(0, b2.solid);
            // DA.SetDataList(1, b2.selectedFragments);
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
                return Properties.Resources.builder;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("ca813b46-dcce-4a49-ae8d-81c7ccb43314"); }
        }
    }
}