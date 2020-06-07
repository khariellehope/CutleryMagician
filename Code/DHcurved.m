D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.0741;
D5 = 0.0741;
D6 = 0.1600;
e2 = 0.0098;

aa = ((30*pi)/180);
sa = sin(aa);
s2a = sin(2*aa);
d4b = D3 + (sa/s2a) * D4;
d5b = (sa/s2a)*D4+(sa/s2a)*D5;
d6b = (sa/s2a)*D5 + D6;

alpha4 = 2*aa;
dlink4 = -d4b;

alpha5 = 2*aa;
dlink5 = -d5b;
dlink6 = -d6b;

