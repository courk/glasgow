from ... import *
from . import EthernetRGMIIApplet


class EthernetRGMIIAppletTestCase(GlasgowAppletTestCase, applet=EthernetRGMIIApplet):
    @synthesis_test
    def test_build(self):
        self.assertBuilds()
