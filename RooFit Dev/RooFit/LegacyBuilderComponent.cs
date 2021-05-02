using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace RooFit
{
    public class LegacyBuilderComponent : GH_Component
    {
        // Input list for the remodeling component
        // 1. Mesh or Point. 
        // 2. Extracted Planes from previous step. 
        // 3. Extrusion, as bounding box and baseline. 

        // New Builder algorithm. We can switch to the old one if this failed. 

        // Using points/ mesh faces to evaluate planes. 
        // Assign each point/ each mesh face to the best fitting plane. 
        // Count which plane is the best one. 
        // Fit the points/ mesh face to the rest of plane. 
        // * Like the reverse process of RANSAC_mesh 
        // Another hyper-parameter. (or adaptive). Top n plane selected. 

        // -- evaluation method:  IDK.. maybe we can simply count the inliers, refit, recount. 
        // Threshold that the point/ meshface counts. 
        // Average Sum of Squared distance for inliers. 

        // -- deal with similar planes. 
        // Find the best plane for every round. Remove the inliers, and move to the next round. 
        // We have done this in RANSAC? why doing it again...
        // This could be the best way to deal with data? 

        // -- 



        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public LegacyBuilderComponent()
          : base("RooFit Builder", "Builder",
              "Solid Builder",
              "RooFit", "Beta")
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

            // 3 - Parameter. 
            pManager.AddNumberParameter("n", "n", "n", GH_ParamAccess.item, 1);
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
            double n = 0;
            DA.GetDataList(0, pts);
            DA.GetData(1, ref extrusion);
            DA.GetDataList(2, planes);
            DA.GetData(3, ref n);

            double tol = 0.001;

            LegacyBuilder lb = new LegacyBuilder(pts, extrusion, planes, n, tol);
            lb.Solve();
            DA.SetData(0, lb.solid);
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
                return Properties.Resources.surface;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("bcdf1fd8-55a6-4d62-8438-0d5ebd2d8638"); }
        }
    }
}