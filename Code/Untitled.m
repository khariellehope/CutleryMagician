 L1 = Link('d', 0.2755, 'a', 0, 'alpha', pi/2, 'offset', 0);
        L2 = Link('d', 0, 'a', 0.4100, 'alpha', pi, 'offset', -pi/2);
        L3 = Link('d', -0.0098, 'a', 0, 'alpha', pi/2, 'offset', pi/2);
        L4 = Link('d', -0.2073, 'a', 0, 'alpha', pi/2, 'offset', 0);
        L5 = Link('d', -0.0743, 'a', 0, 'alpha', pi/2, 'offset', pi);
        L6 = Link('d', -0.1687, 'a', 0, 'alpha', pi, 'offset', 0);
        
        robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'robot');
        workspace = [2 2 2 2 1 -1];
        q = [0,pi,pi,0,0,pi];
        
        robot.plot(q);      %Plot model         
%UR3a.plot(q,'workspace', workspace,'scale',0.25,'trail','r.','fps',50);
robot.teach;
%  L1 = Link('d', 0.2755, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(-10) deg2rad(10)]);
%         L2 = Link('d', 0, 'a', 0.4100, 'alpha', pi, 'offset', 0, 'qlim', [deg2rad(47) deg2rad(313)]);
%         L3 = Link('d', -0.0098, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(19) deg2rad(341)]);
%         L4 = Link('d', -(0.2073+0.0741), 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(-10) deg2rad(10)]);
%         L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(65) deg2rad(295)]);
%         L6 = Link('d', -(0.0741+0.1600), 'a', 0, 'alpha', pi, 'offset', 0, 'qlim', [deg2rad(-10) deg2rad(10)]);
