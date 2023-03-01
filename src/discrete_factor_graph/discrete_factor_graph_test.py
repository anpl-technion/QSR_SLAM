function discreteFactorGraph_test()

close all; clear all; clc;

resdir = 'results_dfg_test';
run_functional_test = false;
run_propagation_test = true;
run_test1 = true;
run_test2 = true;
run_test3 = true;
run_test4 = true;

max_key_id = 10;

debug = true;

if run_functional_test
    fprintf('------------------- functional test -----------------------------\n');
    dfg = discreteFactorGraph(2, max_key_id);
    
    % test add keys
    dfg.addKey(1,[2,3,4,5]);  % wrong size
    dfg.addKey(1,[2,3]);      % valid - with prior
    dfg.addKey(1,[1,2]);      % already exists
    dfg.addKey(2,[]);         % valid - no prior
    dfg.addKey(3);            % valid - no prior
    dfg.addKey(4,[2,1]);      % valid - with prior
    dfg.addKey(5,[2,1]);      % valid - with prior
    disp(dfg);
    
    % test remove keys
    dfg.removeKey(5);
    disp(dfg);
    
    % add factors
    dfg.addFactor(15,[1,5],[1,2,3,4]);  % invalid keys
    dfg.addFactor(12,[1,2],[1,2,3]);  % wrong size
    dfg.addFactor(12,[1,2],[1,2,3,4]);  % valid
    dfg.addFactor(123,[1,2,3],[1,2,3,4,5,6,7,8]);  % valid
    dfg.addFactor(34,[3,4],[1,2,3,4]);  % valid
    disp(dfg);
    
    % remove factors
    dfg.removeFactor(16);  % invalid
    dfg.removeFactor(34);  % valid
    disp(dfg);
    
    dfg.plot();
    
    dfg.write('temp_results','_test1','normal')
    
    dfg2 = discreteFactorGraph(2, max_key_id);
    priors_file = 'temp_results/dfg_priors_test1.txt';
    factors_file = 'temp_results/dfg_factors_test1.txt';
    dfg2.load(priors_file, factors_file, [], 'normal');
    disp(dfg2);
    res = discreteFactorGraph_compare(dfg, dfg2);
    if ~res
        error('comparison failed!');
    end
    
    
    % GTSAM mode
    
    dfg = discreteFactorGraph(2, max_key_id);
    dfg.addKey(1,[2,3]);      % valid - with prior
    dfg.addKey(2,[]);         % valid - no prior
    dfg.addKey(3);            % valid - no prior
    dfg.addKey(4,[2,1]);      % valid - with prior
    dfg.addKey(5,[2,1]);      % valid - with prior
    
    % add factors
    dfg.addFactor([],[1,2,3],[1,2,3,4,5,6,7,8]);
    dfg.addFactor([],[3,4,5],[1,2,3,4,5,6,7,8]);
    dfg.addFactor([],[1,3,5],[1,2,3,4,5,6,7,8]);
    disp(dfg);
    
    dfg.write('temp_results','_test2','GTSAM')
    
    dfg3 = discreteFactorGraph(2, max_key_id);
    priors_file = 'temp_results/dfg_priors_test2.txt';
    factors_file = 'temp_results/dfg_factors_test2.txt';
    factor_potential_file = 'temp_results/dfg_factors_potential_test2.txt';
    dfg3.load(priors_file, [], factors_file, factor_potential_file, 'GTSAM');
    disp(dfg3);
    
    res = discreteFactorGraph_compare(dfg, dfg3);
    if ~res
        error('comparison failed!');
    end
end



if run_propagation_test
    fprintf('------------------- graph propagation test -----------------------------\n');

    figure('name','factor graph propagation');
    ax=gca;
    
    dfg = discreteFactorGraph(2, max_key_id);

    % test add keys
    dfg.addKey(1,[1,4],true);
    dfg.addKey(2,[1,3],true);
    dfg.addKey(3,[],false);
    dfg.addKey(4,[1,0],true);
    dfg.addKey(5,[],false);
    dfg.addKey(6,[],false);
    dfg.addKey(7,[],false);
    dfg.addKey(8,[],false);
    dfg.addKey(9,[],false);

    % add factors
    f = permute(reshape([1,2,3,4,5,6,7,8], [2,2,2]), [3,2,1]);
    dfg.addFactor(123,[1,2,3],f);
    dfg.addFactor(234,[2,3,4],f);
    dfg.addFactor(345,[3,4,5],f);
    dfg.addFactor(567,[5,6,7],f);
    dfg.addFactor(548,[4,5,8],f);
    dfg.plot(ax);

    % solve marginals
    debug = true;
    marginals = dfg.marginals('approx',debug,ax);
    
    %------------------ calc composition level ------------------------
    figure('name','factor graph composition level propagation');
    ax1=gca;
    method = 'normlized';
    dfg.calc_composition_level(method, debug, ax1);

    % plot stuff
    figure('name','composition level Vs composition');
    composition_level = [dfg.keys.composition_level];
    information_score = nan(size(composition_level));
    for ii=1:numel([dfg.keys])
        composition_level(ii) = dfg.keys(ii).composition_level;
        information_score(ii) = dfg.information_score(dfg.keys(ii).marginal);
    end
    plot(composition_level,information_score,'ob');
    
    
    
end



if run_test1
    fprintf('------------------- graph ropagation test 1 - single 2D factor -----------------------------\n');
    test_name = 'test1';
    
    dfg = discreteFactorGraph(2, max_key_id);
    
    % test add keys
    dfg.addKey(1,[0.2,0.8],true);
    dfg.addKey(2,[0.7,0.3],true);
    dfg.addKey(3,[],false);
    
    % add factors
    %     f = permute(reshape([1,2,3,4,5,6,7,8], [2,2,2]), [3,2,1]);
    f = zeros(2,2,2);
    f(1,1,1)=1;
    f(2,1,1)=2;
    f(1,2,1)=3;
    f(2,2,1)=4;
    f(1,1,2)=5;
    f(2,1,2)=6;
    f(1,2,2)=7;
    f(2,2,2)=8;
    f=f./sum(f(:));
    f=permute(f,[3,2,1]);
    dfg.addFactor([],[1,2,3],f);
    
    f1=figure('name','input factor graph');
    dfg.plot(f1);
    
    % solve marginals
    dfg1 = dfg.copy();
    marginals1 = dfg1.marginals('approx',debug);
    f2=figure('name','approx propogated factor graph');
    dfg1.plot(f2,'marginal');
    
    f3=figure('name','brute force propagated factor graph');
    dfg2 = dfg.copy();
    marginals2 = dfg2.marginals('brute force',debug);
    dfg2.plot(f3,'marginal');
    
    % GTSAM
    % dfg.write(resdir,['_',test_name],'GTSAM');
    dfg3 = discreteFactorGraph(2, max_key_id);
    priors_file = fullfile(resdir, sprintf('dfg_priors_%s.txt',test_name));
    factors_file = fullfile(resdir, sprintf('dfg_factors_%s.txt', test_name));
    factor_potential_file = fullfile(resdir, sprintf('dfg_factors_potential_%s.txt', test_name));
    dfg3.load(priors_file, [], factors_file, factor_potential_file, 'GTSAM');
    f4=figure('name','GTSAM propagated factor graph');
    dfg3.plot(f4);
end

if run_test2
    fprintf('------------------- graph ropagation test 2 - single 20D factor -----------------------------\n');
    test_name = 'test2';
    
    xylims=[-2,2,-2,3];
    nsamples=4900;
    T=EdcComposeTable(xylims,nsamples,false);
    
    dfg = discreteFactorGraph(20, max_key_id);
    
    % test add keys
    dfg.addKey(1,[1:20]./sum(1:20),true);
    dfg.addKey(2,[20:-1:1]./sum([20:-1:1]),true);
    dfg.addKey(3,[],false);
    
    % add factors
    % f = permute(reshape([1,2,3,4,5,6,7,8], [2,2,2]), [3,2,1]);
    dfg.addFactor([],[1,2,3],T);
    
    f1=figure('name','input factor graph');
    dfg.plot(f1);
    
    % solve marginals
    dfg1 = dfg.copy();
    marginals1 = dfg1.marginals('approx',debug);
    f2=figure('name','approx propogated factor graph');
    dfg1.plot(f2,'marginal');
    
    f3=figure('name','brute force propagated factor graph');
    dfg2 = dfg.copy();
    marginals2 = dfg2.marginals('brute force',debug);
    dfg2.plot(f3,'marginal');
    
    % GTSAM
    % dfg.write(resdir,['_',test_name],'GTSAM');
    dfg3 = discreteFactorGraph(20, max_key_id);
    priors_file = fullfile(resdir, sprintf('dfg_priors_%s.txt',test_name));
    factors_file = fullfile(resdir, sprintf('dfg_factors_%s.txt', test_name));
    factor_potential_file = fullfile(resdir, sprintf('dfg_factors_potential_%s.txt', test_name));
    dfg3.load(priors_file, [], factors_file, factor_potential_file, 'GTSAM');
    f4=figure('name','GTSAM propagated factor graph');
    dfg3.plot(f4);
end


if run_test3
    fprintf('------------------- graph ropagation test 3 - multiple 2D factors -----------------------------\n');
    test_name = 'test3';
    
    dfg = discreteFactorGraph(2, max_key_id);
    
    % test add keys
    dfg.addKey(1,[0.3,0.7],true);
    dfg.addKey(2,[0.8,0.2],false);
    dfg.addKey(3,[1,0],true);
    dfg.addKey(4,[0,1],false);
    dfg.addKey(5,[],false);
    dfg.addKey(6,[],false);
    
    % add factors
    % f = permute(reshape([1,2,3,4,5,6,7,8], [2,2,2]), [3,2,1]);
    f = zeros(2,2,2);
    f(1,1,1)=1;
    f(2,1,1)=2;
    f(1,2,1)=3;
    f(2,2,1)=4;
    f(1,1,2)=5;
    f(2,1,2)=6;
    f(1,2,2)=7;
    f(2,2,2)=8;
    f=f./sum(f(:));
    dfg.addFactor([],[1,3,5],f);
    dfg.addFactor([],[2,4,5],f);
    dfg.addFactor([],[3,4,5],f);
    dfg.plot();
    
    f1=figure('name','input factor graph');
    dfg.plot(f1);
    
    % solve marginals
    dfg1 = dfg.copy();
    marginals1 = dfg1.marginals('approx',debug);
    f2=figure('name','approx propogated factor graph');
    dfg1.plot(f2,'marginal');
    
    f3=figure('name','brute force propagated factor graph');
    dfg2 = dfg.copy();
    marginals2 = dfg2.marginals('brute force',debug);
    dfg2.plot(f3,'marginal');
    
    % GTSAM
    % dfg.write(resdir,['_',test_name],'GTSAM');
    dfg3 = discreteFactorGraph(2, max_key_id);
    priors_file = fullfile(resdir, sprintf('dfg_priors_%s.txt',test_name));
    factors_file = fullfile(resdir, sprintf('dfg_factors_%s.txt', test_name));
    factor_potential_file = fullfile(resdir, sprintf('dfg_factors_potential_%s.txt', test_name));
    dfg3.load(priors_file, [], factors_file, factor_potential_file, 'GTSAM');
    f4=figure('name','GTSAM propagated factor graph');
    dfg3.plot(f4);
end



if run_test4
    
    fprintf('------------------- graph ropagation test 4 - multiple 20D factors -----------------------------\n');
    test_name = 'test4';
    
    dfg = discreteFactorGraph(20, max_key_id);
    xylims=[-2,2,-2,3];
    nsamples=4900;
    T=EdcComposeTable(xylims,nsamples,false);
    
    % test add keys
    dfg.addKey(1,[1:20]./sum([1:20]),true);
    dfg.addKey(2,[20:-1:1]./sum([20:-1:1]),true);
    dfg.addKey(3,[],false);
    dfg.addKey(4,[1:20]./sum([1:20]),false);
    dfg.addKey(5,[],false);
    
    % add factors
    dfg.addFactor(123,[1,2,3],T);
    dfg.addFactor(234,[2,3,4],T);
    dfg.addFactor(345,[3,4,5],T);
    dfg.plot();
    
    
    f1=figure('name','input factor graph');
    dfg.plot(f1);
    
    % solve marginals
    dfg1 = dfg.copy();
    marginals1 = dfg1.marginals('approx',debug);
    f2=figure('name','approx propogated factor graph');
    dfg1.plot(f2,'marginal');
    
    f3=figure('name','brute force propagated factor graph');
    dfg2 = dfg.copy();
    marginals2 = dfg2.marginals('brute force',debug);
    dfg2.plot(f3,'marginal');
    
    % GTSAM
    dfg.write(resdir,['_',test_name],'GTSAM');
    dfg3 = discreteFactorGraph(20, max_key_id);
    priors_file = fullfile(resdir, sprintf('dfg_priors_%s.txt',test_name));
    factors_file = fullfile(resdir, sprintf('dfg_factors_%s.txt', test_name));
    factor_potential_file = fullfile(resdir, sprintf('dfg_factors_potential_%s.txt', test_name));
    dfg3.load(priors_file, [], factors_file, factor_potential_file, 'GTSAM');
    f4=figure('name','GTSAM propagated factor graph');
    dfg3.plot(f4);
end



%-------------------------- timing --------------------

d = 20;
n_keys = 5;
n_factors = 1;
factor_dim = 3;
fprintf('solving marginals for: %d keys of dim %d and %d factors of %d...\n',n_keys, d, n_factors, factor_dim);

dfg = discreteFactorGraph(d, max_key_id);

for i=1:n_keys
    v = randi(10,[d,1]);
    dfg.addKey(i,v);
end

idx = nchoosek(1:n_keys,factor_dim);
idx2 = randi(size(idx,1), [n_factors,1]);
idx = idx(idx2,:);

for i = 1:n_factors
    f = [1:d^factor_dim];
    dfg.addFactor(i,idx(i,:),f);
end

profile clear;
profile on;
t0 = tic;
marginals = dfg.marginals();
fprintf('time: %f\n',toc(t0));
profile off;
profile viewer
disp('Done');

end

%-------------------------- helper functions ------------------------------
function res = discreteFactorGraph_compare(dfg1, dfg2)
res = true;
for i = 1:numel(dfg1.keys)
    if ~isequal(dfg1.keys(i).id,dfg2.keys(i).id) || sum(abs(dfg1.keys(i).value-dfg2.keys(i).value))>10e-5
        res=false;
    end
end
for i =1:numel(dfg1.factors)
    if ~isequal(dfg1.factors(i).id,dfg2.factors(i).id) ||...
            ~isequal(dfg1.factors(i).keys,dfg2.factors(i).keys) ||...
            sum(abs(dfg1.factors(i).factor(:)-dfg2.factors(i).factor(:)))>10e-5
        res=false;
    end
end

end
