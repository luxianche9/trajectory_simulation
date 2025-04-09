classdef Coordinate < handle
    properties
        Name % 坐标系名称
        Axes % 坐标轴
        RotationMatrix % 旋转矩阵
        BaseCoordinate % 基坐标系, 为另一个Coordinate对象
                       % 在这个坐标系的基础上旋转得到当前坐标系
        AxesHandle
    end

    methods
        function obj = Coordinate(name, rotationMatrix, baseCoordinate)
            obj.Name = name;    
            obj.RotationMatrix = rotationMatrix;
            obj.BaseCoordinate = baseCoordinate;
            obj.Axes = obj.RotationMatrix * obj.BaseCoordinate;
            
            obj.AxesHandle = [];
        end

        function draw(obj, origin, edgeColor, edgeLength, edgeWidth)
            for i = 1:3
                axes = obj.Axes * edgeLength;
                obj.AxesHandle(i) = quiver3(origin(1), origin(2), origin(3), ...
                                            axes(1, i), axes(2, i), axes(3, i), ...
                                            "Color", edgeColor, ...
                                            "LineWidth", edgeWidth);
            end
        end
    end
end