from conans import ConanFile

class CRSGAConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    default_options = {
        "openscenegraph:shared": True
    }

    def requirements(self):
        self.requires("openscenegraph/3.6.5")
