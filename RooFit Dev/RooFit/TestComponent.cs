using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace RooFit
{
    public class TestComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public TestComponent()
          : base("Voting Test", "Voting Test",
              "Voting Test",
              "RooFit", "Dev")
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

            // 3 - Parameter. Top n planes
            pManager.AddNumberParameter("n", "n", "n", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // Final output model 
            pManager.AddBrepParameter("Solid Brep", "Solid Brep", "Solid Brep", GH_ParamAccess.item);

            // Fragment Brep selected to build the model. For dev testing. 
            pManager.AddBrepParameter("Fragments [Dev]", "Fragments [Dev]", "Fragments [Dev]", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

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
                return Properties.Resources.llama;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("7393e3cd-87d3-456f-ab32-92fbcbf31c9b"); }
        }
    }
}