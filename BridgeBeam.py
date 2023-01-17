
import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtil
import GeometryValidate as GeometryValidate

from StdReinfShapeBuilder.RotationAngles import RotationAngles
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties
from HandleService import HandleService

def check_allplan_version(buildEl, version):
    del buildEl
    del version
    return True


def create_element(buildEl, doc):
    element = CreateBridgeBeam(doc)
    return element.create(buildEl)

def move_handle(buildEl, handle_prop, input_pnt, doc):
    buildEl.change_property(handle_prop, input_pnt)
    check_equality(handle_prop.handle_id, buildEl)
    check_Height(buildEl,handle_prop.handle_id)
    return create_element(buildEl, doc)

def check_equality(handle_id,buildEl):
    if (handle_id == "BeamHeight"):
        buildEl.RibHeight.value = buildEl.BeamHeight.value - buildEl.TopShHeight.value - buildEl.BotShLowHeight.value - buildEl.BotShUpHeight.value
        if (buildEl.HoleHeight.value > buildEl.BeamHeight.value - buildEl.TopShHeight.value - 45.5):
            buildEl.HoleHeight.value = buildEl.BeamHeight.value - buildEl.TopShHeight.value - 45.5

def check_Height(buildEl,handle_id):
    if (handle_id == "TopShWidth") or (handle_id == "BotShWidth") or (
            handle_id == "RibThick"):
        temp = min(buildEl.TopShWidth.value, buildEl.BotShWidth.value)
        if (buildEl.RibThick.value >= temp - 100.):
            buildEl.RibThick.value = temp - 100.
        if (buildEl.RibThick.value <= buildEl.VaryingRibThick.value):
            buildEl.VaryingRibThick.value = buildEl.RibThick.value - 20.
        elif (buildEl.RibThick.value - 100. >= buildEl.VaryingRibThick.value):
            buildEl.VaryingRibThick.value = buildEl.RibThick.value - 100.


def change_property(buildEl, name, value):

    if name == "BeamHeight":
        change = value - buildEl.TopShHeight.value - buildEl.RibHeight.value - \
                 buildEl.BotShUpHeight.value - buildEl.BotShLowHeight.value
        print(change)
        change_equality(change, buildEl,value)
    else:
        Switch(buildEl,name,value)

    return True
def change_equality(change,buildEl,value):
    if change < 0:
        change = abs(change)
        if buildEl.TopShHeight.value > 320.:
            if buildEl.TopShHeight.value - change < 320.:
                change -= buildEl.TopShHeight.value - 320.
                buildEl.TopShHeight.value = 320.
            else:
                buildEl.TopShHeight.value -= change
                change = 0.
        if (change != 0) and (buildEl.BotShUpHeight.value > 160.):
            if buildEl.BotShUpHeight.value - change < 160.:
                change -= buildEl.BotShUpHeight.value - 160.
                buildEl.BotShUpHeight.value = 160.
            else:
                buildEl.BotShUpHeight.value -= change
                change = 0.
        if (change != 0) and (buildEl.BotShLowHeight.value > 153.):
            if buildEl.BotShLowHeight.value - change < 153.:
                change -= buildEl.BotShLowHeight.value - 153.
                buildEl.BotShLowHeight.value = 153.
            else:
                buildEl.BotShLowHeight.value -= change
                change = 0.
        if (change != 0) and (buildEl.RibHeight.value > 467.):
            if buildEl.RibHeight.value - change < 467.:
                change -= buildEl.RibHeight.value - 467.
                buildEl.RibHeight.value = 467.
            else:
                buildEl.RibHeight.value -= change
                change = 0.
    else:
        buildEl.RibHeight.value += change
    if value - buildEl.TopShHeight.value - 45.5 < buildEl.HoleHeight.value:
        buildEl.HoleHeight.value = value - buildEl.TopShHeight.value - 45.5

def Switch(buildEl,name,value):
    if name == "TopShHeight":
        buildEl.BeamHeight.value = value + buildEl.RibHeight.value + \
                                     buildEl.BotShUpHeight.value + buildEl.BotShLowHeight.value
    if name == "RibHeight":
        buildEl.BeamHeight.value = value + buildEl.TopShHeight.value + \
                                     buildEl.BotShUpHeight.value + buildEl.BotShLowHeight.value
    if name == "BotShUpHeight":
        buildEl.BeamHeight.value = value + buildEl.TopShHeight.value + \
                                     buildEl.RibHeight.value + buildEl.BotShLowHeight.value
        if value + buildEl.BotShLowHeight.value + 45.5 > buildEl.HoleHeight.value:
            buildEl.HoleHeight.value = value + buildEl.BotShLowHeight.value + 45.5
    if name == "BotShLowHeight":
        buildEl.BeamHeight.value = value + buildEl.TopShHeight.value + \
                                     buildEl.RibHeight.value + buildEl.BotShUpHeight.value
        if buildEl.BotShUpHeight.value + value + 45.5 > buildEl.HoleHeight.value:
            buildEl.HoleHeight.value = buildEl.BotShUpHeight.value + value + 45.5
    if name == "HoleHeight":
        if value > buildEl.BeamHeight.value - buildEl.TopShHeight.value - 45.5:
            buildEl.HoleHeight.value = buildEl.BeamHeight.value - buildEl.TopShHeight.value - 45.5
        elif value < buildEl.BotShLowHeight.value + buildEl.BotShUpHeight.value + 45.5:
            buildEl.HoleHeight.value = buildEl.BotShLowHeight.value + buildEl.BotShUpHeight.value + 45.5
    if name == "HoleDepth":
        if value >= buildEl.BeamLength.value / 2.:
            buildEl.HoleDepth.value = buildEl.BeamLength.value / 2. - 45.5


class CreateBridgeBeam():
    def __init__(self, doc):

        self.model_ele_list = []
        self.handle_list = []
        self.document = doc

    def create(self, buildEl):

        self._top_sh_width = buildEl.TopShWidth.value
        self._top_sh_height = buildEl.TopShHeight.value

        self._bot_sh_width = buildEl.BotShWidth.value
        self._bot_sh_up_height = buildEl.BotShUpHeight.value
        self._bot_sh_low_height = buildEl.BotShLowHeight.value
        self._bot_sh_height = self._bot_sh_up_height + self._bot_sh_low_height

        self._rib_thickness = buildEl.RibThick.value
        self._rib_height = buildEl.RibHeight.value

        self._varying_start = buildEl.VaryingStart.value
        self._varying_length = buildEl.VaryingLength.value
        self._varying_end = self._varying_start + self._varying_length
        self._varying_rib_thickness = buildEl.VaryingRibThick.value

        self._beam_length = buildEl.BeamLength.value
        self._beam_width = max(self._top_sh_width, self._bot_sh_width)
        self._beam_height = buildEl.BeamHeight.value

        self._hole_depth = buildEl.HoleDepth.value
        self._hole_height = buildEl.HoleHeight.value

        self._angleX = buildEl.RotationAngleX.value
        self._angleY = buildEl.RotationAngleY.value
        self._angleZ = buildEl.RotationAngleZ.value

        self.create_beam(buildEl)
        self.create_handles(buildEl)

        AllplanBaseElements.ElementTransform(AllplanGeo.Vector3D(), self._angleX, self._angleY, self._angleZ,
                                             self.model_ele_list)

        rot_angles = RotationAngles(self._angleX, self._angleY, self._angleZ)
        HandleService.transform_handles(self.handle_list, rot_angles.get_rotation_matrix())

        return (self.model_ele_list, self.handle_list)

    def create_beam(self, buildEl):
        com_prop = AllplanBaseElements.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = buildEl.Color3.value
        com_prop.Stroke = 1

        breps = AllplanGeo.BRep3DList()
        bottom_shelf = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., 0., 0.),
                                       AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)),
            self._bot_sh_width / 2., self._beam_length / 2., self._bot_sh_height)

        edges = AllplanUtil.VecSizeTList()
        edges.append(10)
        err, bottom_shelf = AllplanGeo.ChamferCalculus.Calculate(bottom_shelf, edges, 20., False)
        if not GeometryValidate.polyhedron(err):
            return
        breps.append(bottom_shelf)
        top_shelf = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(
            AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0.,
                               self._beam_height - self._top_sh_height), AllplanGeo.Vector3D(1, 0, 0),
            AllplanGeo.Vector3D(0, 0, 1)), self._top_sh_width / 2., self._beam_length / 2., self._top_sh_height)

        top_shelf_notch = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(
            AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0., self._beam_height - 45.),
            AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), 60., self._beam_length / 2., 45.)
        err, top_shelf = AllplanGeo.MakeSubtraction(top_shelf, top_shelf_notch)
        if not GeometryValidate.polyhedron(err):
            return
        breps.append(top_shelf)
        rib = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0., 0., self._bot_sh_height), AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)), self._beam_width / 2., self._beam_length / 2.,
            self._rib_height)
        breps.append(rib)

        err, beam = AllplanGeo.MakeUnion(breps)
        if not GeometryValidate.polyhedron(err):
            return
        breps = AllplanGeo.BRep3DList()
        notch_pol = AllplanGeo.Polyline3D()
        start_point = AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0.,
                                         self._beam_height - self._top_sh_height)
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
        path += AllplanGeo.Point3D(0, self._varying_start, 0) if buildEl.CheckBoxV.value else AllplanGeo.Point3D(0,
                                                                                                                   self._beam_length / 2.,
                                                                                                                   0)

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
        self.varying_notches(self,buildEl,notch_pol,breps,edges)
        self.siling_holes(self, beam, buildEl, breps)
        self.result(self,beam,com_prop)
        
    def varying_notches(self,buildEl,notch_pol,breps,edges):
        if buildEl.CheckBoxV.value:
            profiles = []
            profiles.append(AllplanGeo.Move(notch_pol, AllplanGeo.Vector3D(0, self._varying_start, 0)))

            lines = []
            lines.append(AllplanGeo.Line3D(notch_pol.GetPoint(0), notch_pol.GetPoint(5)))
            lines.append(AllplanGeo.Line3D(notch_pol.GetPoint(1), notch_pol.GetPoint(2)))
            lines.append(AllplanGeo.Move(AllplanGeo.Line3D(notch_pol.GetPoint(0), notch_pol.GetPoint(1)),
                                         AllplanGeo.Vector3D((self._rib_thickness - self._varying_rib_thickness) / 2.,
                                                             0, 0)))
            intersections = [None, None]
            b, intersections[0] = AllplanGeo.IntersectionCalculusEx(lines[0], lines[2])
            b, intersections[1] = AllplanGeo.IntersectionCalculusEx(lines[1], lines[2])

            notch_pol = AllplanGeo.Polyline3D()
            start_point = AllplanGeo.Point3D((self._beam_width - self._varying_rib_thickness) / 2., self._varying_end,
                                             intersections[0].Z)
            notch_pol += start_point
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._varying_rib_thickness) / 2., self._varying_end,
                                            intersections[1].Z)
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., self._varying_end,
                                            self._bot_sh_low_height)
            notch_pol += AllplanGeo.Point3D(-10., self._varying_end, self._bot_sh_low_height)
            notch_pol += AllplanGeo.Point3D(-10., self._varying_end, self._beam_height - 100.)
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., self._varying_end,
                                            self._beam_height - 100.)
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
    def siling_holes(self,beam,buildEl,breps):
        sling_hole = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, buildEl.HoleDepth.value, buildEl.HoleHeight.value),
                                       AllplanGeo.Vector3D(0, 0, 1), AllplanGeo.Vector3D(1, 0, 0)), 45.5,
            self._beam_width)
        breps.append(sling_hole)

        err, beam = AllplanGeo.MakeSubtraction(beam, breps)
        if not GeometryValidate.polyhedron(err):
            return
    def result(self,beam,com_prop):
        plane = AllplanGeo.Plane3D(AllplanGeo.Point3D(self._beam_width / 2., 0, 0), AllplanGeo.Vector3D(1, 0, 0))
        err, beam = AllplanGeo.MakeUnion(beam, AllplanGeo.Mirror(beam, plane))
        if not GeometryValidate.polyhedron(err):
            return
        plane.Set(AllplanGeo.Point3D(0, self._beam_length / 2., 0), AllplanGeo.Vector3D(0, 1, 0))
        err, beam = AllplanGeo.MakeUnion(beam, AllplanGeo.Mirror(beam, plane))
        if not GeometryValidate.polyhedron(err):
            return
        self.model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop, beam))
    def create_handle1(self):
        handle1 = HandleProperties(
            "BeamLength",
            AllplanGeo.Point3D(0., self._beam_length, 0.),
            AllplanGeo.Point3D(0, 0, 0),
            [("BeamLength", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle1)

    def create_handle2(self):
        handle2 = HandleProperties(
            "BeamHeight",
            AllplanGeo.Point3D(0., 0., self._beam_height),
            AllplanGeo.Point3D(0, 0, 0),
            [("BeamHeight", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle2)

    def create_handle3(self):
        handle3 = HandleProperties(
            "TopShWidth",
            AllplanGeo.Point3D(
                (self._beam_width - self._top_sh_width) / 2. + self._top_sh_width, 0., self._beam_height - 45.
            ),
            AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0, self._beam_height - 45.),
            [("TopShWidth", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle3)

    def create_handle4(self):
        handle4 = HandleProperties(
            "BotShWidth",
            AllplanGeo.Point3D(
                (self._beam_width - self._bot_sh_width) / 2. + self._bot_sh_width, 0., self._bot_sh_low_height
            ),

            AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., 0, self._bot_sh_low_height),
            [("BotShWidth", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle4)

    def create_handle5(self):
        handle5 = HandleProperties(
            "RibThick",
            AllplanGeo.Point3D(
                (self._beam_width - self._rib_thickness) / 2. + self._rib_thickness, 0., self._beam_height / 2.
            ),
            AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0, self._beam_height / 2.),
            [("RibThick", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle5)
