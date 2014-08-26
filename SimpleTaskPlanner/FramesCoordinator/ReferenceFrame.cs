using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SimpleTaskPlanner
{
	class ReferenceFrame
	{
		string name;
		ReferenceFrame parent;
		SortedList<string, ReferenceFrame> childs;

		public ReferenceFrame(string Name)
		{
			this.name = Name;
			this.childs = new SortedList<string, ReferenceFrame>();
		}

		public string Name
		{
			get
			{
				return this.name;
			}
		}

		private ReferenceFrame Root
		{
			get
			{
				if (this.parent == null)
					return this;
				else
					return this.parent.Root;
			}
		}

		public void AddChildFrame(ReferenceFrame childFrame, List<HomogeneousTM> Transformation)
		{
			childFrame.parent = this;
			this.childs.Add(childFrame.Name, childFrame);
		}

		public ReferenceFrame SearchChildFrame(string name)
		{
			ReferenceFrame found;
			if (this.name == name)
				return this;
			else
				foreach (ReferenceFrame rf in childs.Values)
				{
					found = rf.SearchChildFrame(name);
					if (found == null)
						continue;
					return found;
				}

			return null;
		}

		public ReferenceFrame SearchRF(string name)
		{
			return this.Root.SearchChildFrame(name);
		}
	}
}
