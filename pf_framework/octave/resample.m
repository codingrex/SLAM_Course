% resample the set of particles.
% A particle has a probability proportional to its weight to get
% selected. A good option for such a resampling method is the so-called low
% variance sampling, Probabilistic Robotics pg. 109
function newParticles = resample(particles)

numParticles = length(particles);

w = [particles.weight];

% normalize the weight
w = w / sum(w);

% consider number of effective particles, to decide whether to resample or not
useNeff = false;
%useNeff = true;
if useNeff
  neff = 1. / sum(w.^2);
  neff
  if neff > 0.5*numParticles
    newParticles = particles;
    for i = 1:numParticles
      newParticles(i).weight = w(i);
    end
    return;
  end
end

newParticles = struct;

% TODO: implement the low variance re-sampling

% the cumulative sum
c= w(1);


% initialize the step and the current position on the roulette wheel
j= 1;
interval= 1 / numParticles;
r= unifrnd(0, interval);


% walk along the wheel to select the particles
for i = 1:numParticles
   U= r + (i - 1) * interval;
   while U > c
     j= j + 1;
     c= c + w(j);
   endwhile
   newParticles(i)= particles(j);
  
end

end
