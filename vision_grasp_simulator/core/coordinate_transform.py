class CoordinateTransformer:
    """像素坐标到虚拟工作台坐标转换。"""
    def __init__(self, pixel_to_mm=1.2, origin_offset=(350,0,0), default_grasp_height=45):
        self.pixel_to_mm=pixel_to_mm; self.origin_offset=origin_offset; self.default_grasp_height=default_grasp_height
    def pixel_to_world(self, px, py, image_shape):
        h,w=image_shape[:2]; ox,oy,oz=self.origin_offset
        x=(px-w/2)*self.pixel_to_mm+ox
        y=(h/2-py)*self.pixel_to_mm+oy
        z=self.default_grasp_height+oz
        return (round(x,2),round(y,2),round(z,2))
