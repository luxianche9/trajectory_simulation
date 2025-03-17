function vec = Euler2Vec(v, theta, phi)
    vec = [v * cos(theta) * cos(phi);
           v * sin(theta);
          -v * cos(theta) * sin(phi)];
end