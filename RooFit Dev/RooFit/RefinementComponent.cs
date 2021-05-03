using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace RooFit
{
    public class RefinementComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public RefinementComponent()
          : base("RooFit Refinement", "Refinement",
              "Refinement",
              "RooFit", "Dev")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // 0 - extrusion. Closed brep
            pManager.AddBrepParameter("Extrusion", "Extrusion", "Extrusion", GH_ParamAccess.item);

            // 1 - list of extracted planes
            pManager.AddPlaneParameter("Planes", "Planes", "Planes", GH_ParamAccess.list);

            // Some parameters for testing
            // 2 - Similarity threshold in xy direction
            pManager.AddNumberParameter("XY Similarity", "XY Similarity", "XY Similarity", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Refiend Planes", "Refined Planes", "Refined Planes", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep extrusion = new Brep() ;
            List<Plane> in_planes = new List<Plane>();
            double xySimilarity = 0;
            double zSimilarity = 0;
            DA.GetData(0, ref extrusion);
            DA.GetDataList(1, in_planes);
            DA.GetData(2, ref xySimilarity);

            Refinement re = new Refinement(extrusion, in_planes, xySimilarity, zSimilarity);
            re.Solve();

            DA.SetDataList(0, re.refinedPlanes);
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
                return Properties.Resources.refinement;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("e3cb11fa-997d-4db3-aac1-a4a0e5897b24"); }
        }
    }
}