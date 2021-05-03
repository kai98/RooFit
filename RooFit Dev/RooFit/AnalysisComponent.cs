using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace RooFit
{
    public class AnalysisComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public AnalysisComponent()
          : base("RooFit Analysis", "Analysis",
              "Analysis Component for RooFit",
              "RooFit", "RooFit")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // 0. Mesh input. 
            pManager.AddMeshParameter("Mesh", "Mesh", "Mesh", GH_ParamAccess.item);

            // 1. Threshold for RANSAC. 
            pManager.AddNumberParameter("Threshold", "Threshold", "Threshold", GH_ParamAccess.item, 0.975);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // 0. Output, List of Extracted Plane. 
            pManager.AddPlaneParameter("Planes", "Planes", "Extracted Planes from points", GH_ParamAccess.list);

            // 1. Output, List of Mesh. List of Mesh can have better visualizations. 
            pManager.AddMeshParameter("Mesh List [Dev]", "Mesh List [Dev]", "Mesh List [Dev]", GH_ParamAccess.list);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh delMesh = null;
            Double threshold = 0.5;

            DA.GetData(0, ref delMesh);
            DA.GetData(1, ref threshold);

            Analysis analysis = new Analysis(delMesh, threshold);
            analysis.Solve();

            DA.SetDataList(0, analysis.resultPlaneList);
            DA.SetDataList(1, analysis.resultMeshList);
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
                return Properties.Resources.analysis;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("91059da1-fb62-4bf0-8778-e34b0b55b2fb"); }
        }
    }
}