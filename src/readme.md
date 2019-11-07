# 新增部分

## 新增文件 dete_circle.py

- 从外向内的扫描
- 含类DeteCircle（继承DETE）
- 修改了DETE的explore(), generate_explore_views(), explore_point()

## 新增文件 dete_circle_binary.py

- 二分
- 含类DeteCircle（继承DETE）

- 与DeteCircle相比，修改了explore()，新增了between()
- 另外，与DeteCircle的generate_explore_views()不同。考虑二分的需求，将同心圆数限制为2^n，目前为4。

## utils.py 中新增函数

用于产生包围待探索点的最小三角形

1. is_ray_intersects_segment()
2. is_poi_within_poly()
3. generate_smallest_triangle()
4. point2list()