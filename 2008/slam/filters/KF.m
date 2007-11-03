k_max = 100;
t = linspace(0,10,k_max);
s = [10*sin(t); 10*cos(t)];  % s(k) is the state at step k
u = [[0;0],  s(:,2:end) - s(:,1:(end-1))]; % u(k) is the controls that go from state k-1 to state k
z = [[0;0],  s(:,2:end)]; % u(k) is the measurments at step k

R = [0, 0;
	 0, 0]; % process noise
Q = [0, 0;
	 0, 0]; % measurment noise

mu = s(:,1);
Sigma = [1, 0;
		 0, 1 ];


for k = 2:k_max

	deltaT = t(k) - t(k-1);
	A = [1, deltaT;
		 0, 1		];
	B = [0 ;
		 1  ];
	C = [1, 0;
		 0, 1 ];

	muPrev = mu(:,(k-1));
	SigmaPrev = Sigma( :, (2*k-3):(2*k-2) );
	u_k = u(k);
	z_k = z(:,k);

	muPriori = A * muPrev + B * u_k;
	SigmaPriori = A * SigmaPrev * A' + R;

	K = SigmaPriori * C' * (C * SigmaPriori * C' + Q)^(-1);
	muNew = muPriori + K * (z_k - C * muPriori);
	SigmaNew = (I - K * C) * SigmaPrev;

	mu(:,k) = muNew;
	Sigma( :, (2*k-1):(2*k) ) = SigmaNew;
end

stateErr = mu - s;
plot(t, stateErr(1,:), ";x;", t, stateErr(2,:), ";v;");
title('Total Error');


