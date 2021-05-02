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

namespace RooFit.Properties
{
    public class PreprocessComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public PreprocessComponent()
          : base("RooFit Preprocess", "Preprocess",
              "Preprocess",
              "RooFit", "Beta")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            //0. Points
            pManager.AddPointParameter("Points", "Points", "Points", GH_ParamAccess.list);

            //1. Footprint
            pManager.AddCurveParameter("Footprint", "Footprint", "Footprint", GH_ParamAccess.item);

            //2. Maximum Height
            pManager.AddNumberParameter("Height", "Height", "Height", GH_ParamAccess.item, 0);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // 0. Points below max-height, if applicable
            pManager.AddPointParameter("Points", "Points", "Points", GH_ParamAccess.list);

            // 1. Bounding Polygon
            pManager.AddBrepParameter("Extrusion", "Extrusion", "Extrusion", GH_ParamAccess.item);

            // 2. Valid Mesh
            pManager.AddMeshParameter("Mesh", "Mesh", "Mesh", GH_ParamAccess.item);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Double tol = 0.001;

            var pts = new List<Point3d>();
            Curve ftCurve = null;
            Double height = 0;

            DA.GetDataList(0, pts);
            DA.GetData(1, ref ftCurve);
            DA.GetData(2, ref height);

            Preprocess prep = new Preprocess(pts, ftCurve, height, tol);
            prep.Solve();

            DA.SetDataList(0, prep.ptInBox);
            DA.SetData(1, prep.extrusion);
            DA.SetData(2, prep.meshInBox);
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
                return Properties.Resources.layers;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("2660F092-F96B-4246-B16D-7A40F8970F1D"); }
        }
    }
}