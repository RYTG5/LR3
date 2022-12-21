
# Script for the usage of the CreateBridgeBeam function

import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtil
import GeometryValidate as GeometryValidate

from StdReinfShapeBuilder.RotationAngles import RotationAngles
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties
from HandleService import HandleService

print('Load BridgeBeam.py')

def check_allplan_version(build_ele, version):    
    # Delete unused arguments
    del build_ele
    del version

    # Support all versions
    return True

def create_element(build_ele, doc):
    # Create Element
    element = CreateBridgeBeam(doc)

    # Return a tuple with elements list and handles list
    return element.create(build_ele)

def move_handle(build_ele, handle_prop, input_pnt, doc):
    # Change the handle properties
    build_ele.change_property(handle_prop, input_pnt)

    if (handle_prop.handle_id == "BeamHeight"):
        build_ele.RibHeight.value = build_ele.BeamHeight.value - build_ele.TopShHeight.value - build_ele.BotShLowHeight.value - build_ele.BotShUpHeight.value
        if (build_ele.HoleHeight.value > build_ele.BeamHeight.value - build_ele.TopShHeight.value - 45.5):
            build_ele.HoleHeight.value = build_ele.BeamHeight.value - build_ele.TopShHeight.value - 45.5
    elif (handle_prop.handle_id == "TopShWidth") or (handle_prop.handle_id == "BotShWidth") or (handle_prop.handle_id == "RibThick"):
        temp = min(build_ele.TopShWidth.value, build_ele.BotShWidth.value)
        if (build_ele.RibThick.value >= temp - 100.):
            build_ele.RibThick.value = temp - 100.
        if (build_ele.RibThick.value <= build_ele.VaryingRibThick.value):
            build_ele.VaryingRibThick.value = build_ele.RibThick.value - 20.
        elif (build_ele.RibThick.value - 100. >= build_ele.VaryingRibThick.value):
            build_ele.VaryingRibThick.value = build_ele.RibThick.value - 100.

    # Recreate element with new properties
    return create_element(build_ele, doc)

def modify_element_property(build_ele, name, value):
    # Handle dependencies for changed property
    # Return true/false for palette refresh
    if (name == "BeamHeight"):
        change = value - build_ele.TopShHeight.value - build_ele.RibHeight.value - build_ele.BotShUpHeight.value - build_ele.BotShLowHeight.value
        print(change)
        if (change < 0):
            change = abs(change)
            if (build_ele.TopShHeight.value > 320.):
                if (build_ele.TopShHeight.value - change < 320.):
                    change -= build_ele.TopShHeight.value - 320.
                    build_ele.TopShHeight.value = 320.
                else:
                    build_ele.TopShHeight.value -= change
                    change = 0.
            if (change != 0) and (build_ele.BotShUpHeight.value > 160.):
                if (build_ele.BotShUpHeight.value - change < 160.):
                    change -= build_ele.BotShUpHeight.value - 160.
                    build_ele.BotShUpHeight.value = 160.
                else:
                    build_ele.BotShUpHeight.value -= change
                    change = 0.
            if (change != 0) and (build_ele.BotShLowHeight.value > 153.):
                if (build_ele.BotShLowHeight.value - change < 153.):
                    change -= build_ele.BotShLowHeight.value - 153.
                    build_ele.BotShLowHeight.value = 153.
                else:
                    build_ele.BotShLowHeight.value -= change
                    change = 0.
            if (change != 0) and (build_ele.RibHeight.value > 467.):
                if (build_ele.RibHeight.value - change < 467.):
                    change -= build_ele.RibHeight.value - 467.
                    build_ele.RibHeight.value = 467.
                else:
                    build_ele.RibHeight.value -= change
                    change = 0.
        else:
            build_ele.RibHeight.value += change
        if (value - build_ele.TopShHeight.value - 45.5 < build_ele.HoleHeight.value):
            build_ele.HoleHeight.value = value - build_ele.TopShHeight.value - 45.5   
    elif (name == "TopShHeight"):
        build_ele.BeamHeight.value = value + build_ele.RibHeight.value + build_ele.BotShUpHeight.value + build_ele.BotShLowHeight.value
    elif (name == "RibHeight"):
        build_ele.BeamHeight.value = value + build_ele.TopShHeight.value + build_ele.BotShUpHeight.value + build_ele.BotShLowHeight.value
    elif (name == "BotShUpHeight"):
        build_ele.BeamHeight.value = value + build_ele.TopShHeight.value + build_ele.RibHeight.value + build_ele.BotShLowHeight.value
        temp = value + build_ele.BotShLowHeight.value + 45.5
        if (temp > build_ele.HoleHeight.value):
            build_ele.HoleHeight.value = temp
    elif (name == "BotShLowHeight"):
        build_ele.BeamHeight.value = value + build_ele.TopShHeight.value + build_ele.RibHeight.value + build_ele.BotShUpHeight.value
        temp = build_ele.BotShUpHeight.value + value + 45.5
        if (temp > build_ele.HoleHeight.value):
            build_ele.HoleHeight.value = temp
    elif (name == "HoleHeight"):
        temp = build_ele.BeamHeight.value - build_ele.TopShHeight.value - 45.5
        temp1 = build_ele.BotShLowHeight.value + build_ele.BotShUpHeight.value + 45.5
        if (value > temp):
            build_ele.HoleHeight.value = temp
        elif (value < temp1):
            build_ele.HoleHeight.value = temp1
    elif (name == "HoleDepth") and (value >= build_ele.BeamLength.value / 2.):
        build_ele.HoleDepth.value = build_ele.BeamLength.value / 2. - 45.5
    elif (name == "TopShWidth") or (name == "BotShWidth") or (name == "RibThick"):
        temp = min(build_ele.TopShWidth.value, build_ele.BotShWidth.value)
        if (build_ele.RibThick.value >= temp - 100.):
            build_ele.RibThick.value = temp - 100.
        if (value <= build_ele.VaryingRibThick.value):
            build_ele.VaryingRibThick.value = build_ele.RibThick.value - 20.
        elif (value - 100. >= build_ele.VaryingRibThick.value):
            build_ele.VaryingRibThick.value = build_ele.RibThick.value - 100.
    elif (name == "VaryingStart"):
        temp = build_ele.BeamLength.value / 2.
        if (value >= temp):
            build_ele.VaryingStart.value = temp - 200.
        temp -= build_ele.VaryingStart.value
        if (build_ele.VaryingLength.value >= temp):
            build_ele.VaryingLength.value = temp - 100.
    elif (name == "VaryingLength"):
        temp = build_ele.BeamLength.value / 2. - build_ele.VaryingStart.value
        if (value >= temp):
            build_ele.VaryingLength.value = temp - 100.
    elif (name == "VaryingRibThick"):
        if (value >= build_ele.RibThick.value):
            build_ele.VaryingRibThick.value = build_ele.RibThick.value - 20.
        elif (value <= build_ele.RibThick.value - 100.):
            build_ele.VaryingRibThick.value = build_ele.RibThick.value - 100.

    return True

class CreateBridgeBeam():
    def __init__(self, doc):

        self.model_ele_list = []
        self.handle_list = []
        self.document = doc
        
    def create(self, build_ele):
                
        self._top_sh_width = build_ele.TopShWidth.value
        self._top_sh_height = build_ele.TopShHeight.value

        self._bot_sh_width = build_ele.BotShWidth.value
        self._bot_sh_up_height = build_ele.BotShUpHeight.value
        self._bot_sh_low_height = build_ele.BotShLowHeight.value
        self._bot_sh_height = self._bot_sh_up_height + self._bot_sh_low_height
        
        self._rib_thickness = build_ele.RibThick.value
        self._rib_height = build_ele.RibHeight.value

        self._varying_start = build_ele.VaryingStart.value
        self._varying_length = build_ele.VaryingLength.value
        self._varying_end = self._varying_start + self._varying_length
        self._varying_rib_thickness = build_ele.VaryingRibThick.value

        self._beam_length = build_ele.BeamLength.value
        self._beam_width = max(self._top_sh_width, self._bot_sh_width)
        self._beam_height = build_ele.BeamHeight.value

        self._hole_depth = build_ele.HoleDepth.value
        self._hole_height = build_ele.HoleHeight.value

        self._angleX = build_ele.RotationAngleX.value
        self._angleY = build_ele.RotationAngleY.value
        self._angleZ = build_ele.RotationAngleZ.value

        self.create_beam(build_ele)
        self.create_handles(build_ele)
        
        AllplanBaseElements.ElementTransform(AllplanGeo.Vector3D(), self._angleX, self._angleY, self._angleZ, self.model_ele_list)

        rot_angles = RotationAngles(self._angleX, self._angleY, self._angleZ)
        HandleService.transform_handles(self.handle_list, rot_angles.get_rotation_matrix())
        
        return (self.model_ele_list, self.handle_list)

    def create_beam(self, build_ele):
        com_prop = AllplanBaseElements.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = build_ele.Color3.value
        com_prop.Stroke = 1

        breps = AllplanGeo.BRep3DList()
        # bottom shelf
        bottom_shelf = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., 0., 0.), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), self._bot_sh_width / 2., self._beam_length / 2., self._bot_sh_height)

        edges = AllplanUtil.VecSizeTList()
        edges.append(10)
        err, bottom_shelf = AllplanGeo.ChamferCalculus.Calculate(bottom_shelf, edges, 20., False)
        if not GeometryValidate.polyhedron(err):
            return        
        breps.append(bottom_shelf)
        
        # top shelf
        top_shelf = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0., self._beam_height - self._top_sh_height), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), self._top_sh_width / 2., self._beam_length / 2., self._top_sh_height)

        top_shelf_notch = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0., self._beam_height - 45.), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), 60., self._beam_length  / 2., 45.)
        err, top_shelf = AllplanGeo.MakeSubtraction(top_shelf, top_shelf_notch)
        if not GeometryValidate.polyhedron(err):
            return
        breps.append(top_shelf)
        
        # rib
        rib = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0., 0., self._bot_sh_height), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), self._beam_width / 2., self._beam_length / 2., self._rib_height)
        breps.append(rib)
        
        err, beam = AllplanGeo.MakeUnion(breps)
        if not GeometryValidate.polyhedron(err):
            return

        # notches
        breps = AllplanGeo.BRep3DList()                
        notch_pol = AllplanGeo.Polyline3D()
        start_point = AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0., self._beam_height - self._top_sh_height)
        notch_pol += start_point
        notch_pol += AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0., self._bot_sh_height)
        notch_pol += AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., 0., self._bot_sh_low_height)
        notch_pol += AllplanGeo.Point3D(-10., 0., self._bot_sh_low_height)     
        notch_pol += AllplanGeo.Point3D(-10., 0., self._beam_height - 100.)
        notch_pol += AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0., self._beam_height - 100.)
        notch_pol += start_point
        if not GeometryValidate.is_valid(notch_pol):
            return

        path = AllplanGeo.Polyline3D()
        path += AllplanGeo.Point3D(0, 0, 0)
        path += AllplanGeo.Point3D(0, self._varying_start, 0) if build_ele.CheckBoxV.value else AllplanGeo.Point3D(0, self._beam_length / 2., 0)
        
        err, notch = AllplanGeo.CreateSweptBRep3D(notch_pol, path, False, None)
        if not GeometryValidate.polyhedron(err):
            return
        edges = AllplanUtil.VecSizeTList()
        edges.append(3)
        edges.append(1)
        err, notch = AllplanGeo.FilletCalculus3D.Calculate(notch, edges, 100., False)
        if not GeometryValidate.polyhedron(err):
            return
        breps.append(notch)

        # varying profile notches
        if build_ele.CheckBoxV.value:
            profiles = []
            profiles.append(AllplanGeo.Move(notch_pol, AllplanGeo.Vector3D(0, self._varying_start, 0)))
                    
            lines = []
            lines.append(AllplanGeo.Line3D(notch_pol.GetPoint(0), notch_pol.GetPoint(5)))
            lines.append(AllplanGeo.Line3D(notch_pol.GetPoint(1), notch_pol.GetPoint(2)))
            lines.append(AllplanGeo.Move(AllplanGeo.Line3D(notch_pol.GetPoint(0), notch_pol.GetPoint(1)), AllplanGeo.Vector3D((self._rib_thickness - self._varying_rib_thickness) / 2., 0, 0)))
            intersections = [None,None]
            b, intersections[0] = AllplanGeo.IntersectionCalculusEx(lines[0], lines[2])
            b, intersections[1] = AllplanGeo.IntersectionCalculusEx(lines[1], lines[2])
                        
            notch_pol = AllplanGeo.Polyline3D()
            start_point = AllplanGeo.Point3D((self._beam_width - self._varying_rib_thickness) / 2., self._varying_end, intersections[0].Z)
            notch_pol += start_point
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._varying_rib_thickness) / 2., self._varying_end, intersections[1].Z)
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., self._varying_end, self._bot_sh_low_height)
            notch_pol += AllplanGeo.Point3D(-10., self._varying_end, self._bot_sh_low_height)     
            notch_pol += AllplanGeo.Point3D(-10., self._varying_end, self._beam_height - 100.)
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., self._varying_end, self._beam_height - 100.)
            notch_pol += start_point
            if not GeometryValidate.is_valid(notch_pol):
                return
            
            path = AllplanGeo.Polyline3D()
            path += AllplanGeo.Point3D(0, self._varying_end, 0)
            path += AllplanGeo.Point3D(0, self._beam_length / 2., 0)

            err, notch = AllplanGeo.CreateSweptBRep3D(notch_pol, path, False, None)
            if not GeometryValidate.polyhedron(err):
                return
            err, notch = AllplanGeo.FilletCalculus3D.Calculate(notch, edges, 100., False)
            if not GeometryValidate.polyhedron(err):
                return
            breps.append(notch)

            profiles.append(notch_pol)
            path = AllplanGeo.Line3D(profiles[0].GetStartPoint(), profiles[1].GetStartPoint())

            err, notch = AllplanGeo.CreateRailSweptBRep3D(profiles, [path], True, False, False)

            edges = AllplanUtil.VecSizeTList()
            edges.append(11)
            edges.append(9)
            err, notch = AllplanGeo.FilletCalculus3D.Calculate(notch, edges, 100., False)
            if not GeometryValidate.polyhedron(err):
                return
            breps.append(notch)

        # sling_holes
        sling_hole = AllplanGeo.BRep3D.CreateCylinder(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0,build_ele.HoleDepth.value, build_ele.HoleHeight.value), AllplanGeo.Vector3D(0, 0, 1), AllplanGeo.Vector3D(1, 0, 0)), 45.5, self._beam_width)
        breps.append(sling_hole)
        
        err, beam = AllplanGeo.MakeSubtraction(beam, breps)
        if not GeometryValidate.polyhedron(err):
            return

        # result beam
        plane = AllplanGeo.Plane3D(AllplanGeo.Point3D(self._beam_width / 2., 0, 0), AllplanGeo.Vector3D(1, 0, 0))
        err, beam = AllplanGeo.MakeUnion(beam, AllplanGeo.Mirror(beam, plane))
        if not GeometryValidate.polyhedron(err):
            return
        plane.Set(AllplanGeo.Point3D(0, self._beam_length / 2., 0), AllplanGeo.Vector3D(0, 1, 0))
        err, beam = AllplanGeo.MakeUnion(beam, AllplanGeo.Mirror(beam, plane))
        if not GeometryValidate.polyhedron(err):
            return
        self.model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop, beam))

    def create_handles(self, build_ele):        
        # Define handles
        handle1 = HandleProperties("BeamLength",
                                   AllplanGeo.Point3D(0., self._beam_length, 0.),
                                   AllplanGeo.Point3D(0, 0, 0),
                                   [("BeamLength", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle1)

        handle2 = HandleProperties("BeamHeight",
                                   AllplanGeo.Point3D(0., 0., self._beam_height),
                                   AllplanGeo.Point3D(0, 0, 0),
                                   [("BeamHeight", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle2)
        
        handle3 = HandleProperties("TopShWidth",
                                   AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2. + self._top_sh_width, 0., self._beam_height - 45.),
                                   AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0, self._beam_height - 45.),
                                   [("TopShWidth", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle3)

        handle4 = HandleProperties("BotShWidth",
                                   AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2. + self._bot_sh_width, 0., self._bot_sh_low_height),
                                   AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., 0, self._bot_sh_low_height),
                                   [("BotShWidth", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle4)
        
        handle5 = HandleProperties("RibThick",
                                   AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2. + self._rib_thickness, 0., self._beam_height / 2.),
                                   AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0, self._beam_height / 2.),
                                   [("RibThick", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle5)

        