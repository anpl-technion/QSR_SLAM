classdef discreteFactorGraph < handle
    %% this is a discrete factor graph
    % it can infere marginals by brute force approach
    
    properties
        dim % discrete node dimention
        keys = struct('id',{},'value',{},'is_oracle',{},'marginal',{},'connected_factors',{},'composition_level',{});
        factors = struct('id',{},'keys',{},'factor',{});
        G = graph();
        max_key_id % maximal key id - for generating factor ids
        marginals_probabilities
     end
    
    methods
        function obj = discreteFactorGraph(discrete_dim, max_key_id)
            %%Class constructor creates objects
            % 
            obj.dim=discrete_dim;
            obj.max_key_id=max_key_id;
        end
        
        function res = copy(obj)
            res = discreteFactorGraph(obj.dim, obj.max_key_id);
            res.keys = obj.keys;
            res.factors = obj.factors;
            res.G = obj.G;
        end
        
        function [d]=get.dim(obj)
            d=obj.dim;
        end
        
        function [nodes]=get.keys(obj)
            nodes=obj.keys;
        end

        function [factors]=get.factors(obj)
            factors=obj.factors;
        end
        
        function res = addKey(obj, id, prior, is_oracle)
            %% add discrete node
            res = 0;
            
            if nargin<4 || isempty(is_oracle)
                is_oracle = false;
            end            
            if nargin<3 || isempty(prior)
                prior = ones(obj.dim,1);
            end
            if id > obj.max_key_id
                error('max key id is %d',obj.max_key_id);
            end

            prior = prior(:)./sum(prior(:));
            if isequal(size(prior), [obj.dim,1])
                ids = [obj.keys.id];
                if ~ismember(id,ids)
                    obj.keys(end+1) = struct('id',id,'value',prior,'is_oracle',is_oracle,'marginal',[],'connected_factors',[],'composition_level',[]);
                    obj.G=obj.G.addnode(num2str(id));
                    res = 1;
                else
                    warning('id %d already exists! not adding node!',id);
                end               
            else
                warning('invalid prior dimentions');
            end

        end
        
        function res = removeKey(obj, id)
            %% add discrete node
            res = 0;
            ids = [obj.keys.id];
            [ia, idx] = ismember(id,ids);
            if ~ia
                warning('id %d not found!',id);
            else
                obj.keys(idx) = [];
                obj.G=obj.G.rmnode(num2str(id));
                for i=1:numel(obj.factors)
                   if ismember(id, obj.factor(i).keys)
                       obj.removeFactor(obj.factor(i).id)
                   end
                end
                res = 1;
            end
        end
        
        function [res] = get_factor_id(obj, keyids)
            scl = 10^ceil(log10(obj.max_key_id+1));
            scl_vector = scl.^(numel(keyids)-1:-1:0);
            res = sum(keyids.*scl_vector);
        end
                    
        function [id] = addFactor(obj, id, keyids, value)
            %% add discrete factor
            % id - numeric integer factor id
            % keysids - key ids (array of numeric integers)
            % value - factor table size: dim ^ num keys
            ids = [obj.factors.id];
            
            if isempty(id)
                id = obj.get_factor_id(keyids);
            end
            
            if ismember(id,ids)
                warning('factor %d already exist!',id);
            elseif ~all(ismember(keyids,[obj.keys.id]))
                warning('invalid key ids!');
            else
                n = numel(keyids);
                vaule = value(:)./sum(value(:));
                all_key_ids = [obj.keys.id];
                if numel(vaule) ~= obj.dim^n
                    warning('invalid factor value! factor not added!'); 
                else
%                     obj.factors(end+1) = struct('id',id,'keys',keyids,'factor',permute(reshape(vaule, ones(1,n)*obj.dim), [n:-1:1]));
                    obj.factors(end+1) = struct('id',id,'keys',keyids,'factor',permute(reshape(vaule, ones(1,n)*obj.dim), [1:n]));
                    fname = sprintf('f:%d',id);
                    obj.G=obj.G.addnode(fname);
                    for i=1:numel(keyids)
                        obj.G=obj.G.addedge(fname, num2str(keyids(i)));
                    end
                    
                    % update corresponding keys
                    for ki=keyids
                        idx = [obj.keys.id]==ki;
                        obj.keys(idx).connected_factors = [obj.keys(idx).connected_factors, id];
                    end

                end
            end
        end
        
        function res = removeFactor(obj, id)
            %% add discrete node
            ids = [obj.factors.id];
            [ia, idx] = ismember(id,ids);
            res = 0;
            if ~ia
                warning('id %d not found!',id);
            else
                keyids = obj.factors(idx).keyids;
                
                % remove factor
                obj.factors(idx) = [];
                fname = sprintf('f:%d',id);
                obj.G = obj.G.rmnode(fname);
                res = 1;
                
                % update corresponding keys
                for ki=keyids
                    idx0 = [obj.keys.id]==ki;
                    obj.keys(idx0).connected_factors = obj.keys(idx0).connected_factors(obj.keys(idx0).connected_factors ~= id);
                end
                
            end
        end
        
        
        function H = vector_entropy(obj, v)
            %% calc vector entropy
            x = v(v>0);
            H = -sum(x.*log(x));
        end        

        function isc = information_score(obj, v)
            %% calc information score
            d=numel(v);
            v0=ones(d,1)/d;
            min_entropy=obj.vector_entropy(v0);
            v0=zeros(d,1); v0(1)=1;
            max_entropy=obj.vector_entropy(v0);
            isc = (obj.vector_entropy(v)-min_entropy)./(max_entropy-min_entropy);
        end        
        
        %------------------------------------------------------------------
        function disp(obj)
            %% Overload disp function. Display objects as output of char method.
            fprintf('discrete factor graph:\n');
            % nodes
            fprintf('%d nodes: ', numel(obj.keys));
            fprintf('%d, ',[obj.keys.id])
            fprintf('\n');
            % oracle nodes
            idx = [obj.keys.is_oracle];
            fprintf('%d oracle nodes: ', numel(obj.keys(idx)));
            fprintf('%d, ',[obj.keys(idx).id])
            fprintf('\n');
            
            % factors
            fprintf('%d factors: ', numel(obj.factors));
            for i=1:numel(obj.factors)
                fprintf('(%d:',obj.factors(i).id);
                for j=1:numel(obj.factors(i).keys())
                    fprintf('%d,',obj.factors(i).keys(j));
                end
                fprintf('), ');
            end
            fprintf('\n');            
        end

        
        %------------------------------------------------------------------
        function [marginals] = marginals(obj, method, debug, ax, save_video, video_object)
            %% calc factor graph marginals for all keys.
            % method - "brute force" - go over all possibilities 
            %                          can't hold a big graph (makes dim^num nodes iterations)
            %          "approx" - uses approximated graph propagation
            if nargin<3
               debug=false; 
            end
            if nargin<4
                ax=[];
            end
            if debug && isempty(ax)
                figure('name','factor grapg propagation');
                ax=gca;
            end
            
            if ~debug || nargin<5
                save_video=false;
            end
            
            d = obj.dim;
            n_nodes = numel(obj.keys);
            n_factors = numel(obj.factors);

            nodeids = [obj.keys.id];
            marginals = zeros(d, n_nodes);

            switch method
                
                case "brute force"
                    % get all node realizations (nrStates^n_nodes)
                    %             C = cell(n_nodes,1);
                    %             for i=1:n_nodes
                    %                 C{i} = 0:d-1;
                    %             end
                    %             allPosbValues = combvec(C{:})';
                    
                    %             C2 = cell(n_nodes,1);
                    %             [C2{:}] = ind2sub(ones(n_nodes)*d,1:n);
                    
                    n = d^n_nodes;
                    C2 = cell(n_nodes,1);
                    check_points = floor(linspace(1,n,10));
                    for i=1:n
                        if debug
                            if ismember(i,check_points)
                                fprintf('%d out of %d\n',i,n);
                            end
                        end
                        
                        % x =  allPosbValues(i,:);
                        [C2{:}] = ind2sub(ones(n_nodes)*d,i);
                        x = cell2mat(C2)-1;
                        
                        % for each realization of all of the nodes, calc probability
                        p_factors = zeros(n_factors,1);
                        for f = 1:n_factors
                            [~,idx] = ismember(obj.factors(f).keys, [obj.keys.id]);
                            C = num2cell(x(idx)+1);
                            % sub2ind([2,2,2],1,1,2)
                            p_factors(f) = obj.factors(f).factor(C{:});
                        end
                        
                        p_priors = zeros(n_nodes,1);
                        for k = 1:n_nodes
                            p_priors(k) = obj.keys(k).value(x(k)+1);
                        end
                        
                        px = prod(p_factors) * prod(p_priors);
                        
                        % add probability to relevant states in each node
                        for j=1:n_nodes
                            marginals(x(j)+1,j) = marginals(x(j)+1,j) + px;
                        end
                        
                    end
                    
                    % update graph nodes
                    for i=1:n_nodes
                        obj.keys(i).marginal = marginals(:,i)./sum(marginals(:,i));
                    end

                    
                case "approx"
                    % approximated algorithm:
                    % 1) mark oracles as updated
                    %
                    % 2) calc all updatable factors:
                    %    for each factors that have updated + non updated
                    %    nodes, calc posterior probability for non updated
                    %    nodes (but don't update yet)
                    %
                    % 3) update most informative factor:
                    %    choose most informative new posterior, and update 
                    %     corresponding node
                    %
                    % 4) continue updates untill all nodes are updated or 
                    %    no more updatable nodes 
                    
                    
                    % mark observed nodes as updated
                    prev_is_updated = [];
                    propagation_step = 0;
                    is_updated = [obj.keys.is_oracle];
                    for i=1:numel(obj.keys)
                        obj.keys(i).marginal = obj.keys(i).value;
                    end

                    if debug
                        title(ax, sprintf('propagate step 0\n'));
                        if save_video
                            F = getframe(ax);
                            writeVideo(video_object,F);
                        end                        
                        
                        [plt,~, xy_nodes, xy_factors]=obj.plot(ax,'value');
                        hold(ax,'on');
                        x = xy_nodes(:,1);
                        y = xy_nodes(:,2);
                        p1=plot(ax,x(is_updated==1),y(is_updated==1),'o','MarkerEdgeColor',[0,0.8,0],'MarkerFaceColor',[0,0.8,0],'MarkerSize',14);
                        title(ax, sprintf('propagate step 0\n'));
                        hold(ax,'off');
                        p2=[];
                        
                        if save_video
                            F = getframe(ax);
                            writeVideo(video_object,F);
                        end
                    end                    
                    
                    % stop when all nodes are updated, or no new nodes updated in last step
                    is_key_calculated = zeros(n_nodes,1);
                    key_calculated_value = nan(d,n_nodes);
                    key_calculated_score = nan(n_nodes,1);
                    while ~all(is_updated) && ~isequal(is_updated,prev_is_updated)  % loop untill all nodes are updated, or untill no change
                        prev_is_updated = is_updated;
                        
                        % find which nodes are already updated
                        updated_node_ids = nodeids(is_updated);
                        nonupdated_node_ids = nodeids(~is_updated);
                        
                        % calculate all updatable factors (factors with updated and non-updated nodes)
                        %    calc marginal for npn-updated factors that are 
                        %    connected to an updatable factor
                        for i=1:numel([obj.factors])
                            [C1, ~, ~] = intersect(obj.factors(i).keys, nonupdated_node_ids);
                            [C2, ~, ~] = intersect(obj.factors(i).keys, updated_node_ids);
                            if numel(C1)==1   % one nonupdated node in the factor
                                idx0 = nodeids==C1;
                                % idx1 = nodeids==C2(1);
                                % idx2 = nodeids==C2(2);
                                % 1->2 propagate
                                [res_nodeids, val] = obj.propagate_factor(obj.factors(i).id);
                                v = val(:,res_nodeids==C1);
                                c = obj.information_score(v);
                                if ~is_key_calculated(idx0) || (is_key_calculated(idx0) && c > key_calculated_score(idx0))
                                    is_key_calculated(idx0) = true;
                                    key_calculated_value(:,idx0) = v;
                                    key_calculated_score(idx0) = c;
                                end
                            end
                            
                            if numel(C1)==2   % two nonupdated nodes in the factor
                                unobserved_node_idx1 = nodeids==C1(1);
                                unobserved_node_idx2 = nodeids==C1(2);
                                % observed_node_idx = nodeids==C2;
                                % 2 -> 1 propagate
                                [res_nodeids, val] = obj.propagate_factor(obj.factors(i).id);
                                v1 = val(:,res_nodeids==C1(1));
                                c1 = obj.information_score(v1);
                                if ~is_key_calculated(unobserved_node_idx1) || (is_key_calculated(unobserved_node_idx1) && c1 > key_calculated_score(unobserved_node_idx1))
                                    is_key_calculated(unobserved_node_idx1) = true;
                                    key_calculated_value(:,unobserved_node_idx1) = v1;
                                    key_calculated_score(unobserved_node_idx1) = c1;
                                end
                                v2 = val(:,res_nodeids==C1(2));
                                c2 = obj.information_score(v2);
                                if ~is_key_calculated(unobserved_node_idx2) || (is_key_calculated(unobserved_node_idx2) && c2 > key_calculated_score(unobserved_node_idx2))
                                    is_key_calculated(unobserved_node_idx2) = true;
                                    key_calculated_value(:,unobserved_node_idx2) = v2;
                                    key_calculated_score(unobserved_node_idx2) = c2;
                                end
                            end
                        end
                        
                        % update most informative non-updated node (best score)
                        
                        [max_score,idx_max] = max(key_calculated_score);
                        if ~isnan(max_score)
                            obj.keys(idx_max).marginal = key_calculated_value(:,idx_max);
                            is_updated(idx_max) = true;
                            
                            % clear connected calculated values
                            % (keep unused calculations for later use)
                            is_key_calculated(idx_max) = false;
                            max_key_id = nodeids(idx_max);
                            connected_factor_ids = obj.keys(idx_max).connected_factors;  % get all connected factors
                            for fi = connected_factor_ids
                                fidx = [obj.factors.id]==fi;
                                [isa, locb] = ismember(nodeids, obj.factors(fidx).keys);  % get all connected keys
                                connected_key_ids = nodeids(isa);
                                for ki = connected_key_ids
                                    idx_tmp = nodeids == ki;
                                    is_key_calculated(idx_tmp) = false;
                                    key_calculated_value(:,idx_tmp) = nan;
                                    key_calculated_score(idx_tmp) = nan;
                                end
                            end
                        end
                        
                        % plot
                        if debug
                            fprintf('propagating node %d\n',max_key_id);
                            propagation_step = propagation_step + 1;
                            
                            [sum(is_key_calculated), sum(is_updated)]
                            
                            if ~isempty(p1)
                                delete(p1);
                            end
                            if ~isempty(p2)
                                delete(p2);
                            end
                            [plt,~, xy_nodes, xy_factors]=obj.plot(ax,'marginal');

                            hold(ax,'on');
                            x = xy_nodes(:,1);
                            y = xy_nodes(:,2);
                            p1=plot(ax,x(is_key_calculated==1),y(is_key_calculated==1),'o','MarkerEdgeColor','r','MarkerFaceColor','none','MarkerSize',14);
                            p2=plot(ax,x(is_updated==1),y(is_updated==1),'o','MarkerEdgeColor',[0,0.8,0],'MarkerFaceColor',[0,0.8,0],'MarkerSize',14);
                            title(ax, sprintf('propagate step %d\n',propagation_step));
                            
                            hold(ax,'off');
                            
                            if save_video
                                F = getframe(ax);
                                writeVideo(video_object,F);
                            end                            
                        end
                        
                    end
                    
                    for i=1:numel(obj.keys)
                        marginals(:,i) = obj.keys(i).marginal(:);
                    end
                                       
                otherwise
                    error('invalig marginals calculation method!');
                    
            end
            
            % normlize
            for i=1:n_nodes
                marginals(:,i) = marginals(:,i)./sum(marginals(:,i));
            end
        end
        
        function [nodeids, val] = propagate_factor(obj, factorid)
            %% propagate data through a single factor
            % get factor
            ids = [obj.factors.id];
            [ia, idx] = ismember(factorid,ids);
            if ~ia
                warning('factor %d not found!',id);
                nodeids=[];
                val=[];
            else
                nodeids = obj.factors(idx).keys;
                d = numel(nodeids);
                [ia2, idx2] = ismember(nodeids, [obj.keys.id]);
                if ~all(ia2)
                    error('nodeid not found!');
                end
                
                switch d
                    case 2
                        v1 = permute(obj.keys(idx2(1)).marginal(:),[1,2]);
                        v2 = permute(obj.keys(idx2(2)).marginal(:),[2,1]);
                        M = v1.*v2;
                        T = M.*obj.factors(idx).factor;
                        val1 = sum(T,1);
                        val2 = sum(T,2);
                        val1 = val1./sum(val1);
                        val2 = val2./sum(val2);                        
                        val = [val1(:),val2(:)];
                        
                    case 3
                        v1 = permute(obj.keys(idx2(1)).marginal(:),[1,2,3]);
                        v2 = permute(obj.keys(idx2(2)).marginal(:),[3,1,2]);
                        v3 = permute(obj.keys(idx2(3)).marginal(:),[2,3,1]);
                        M = v1.*v2.*v3;
                        T = M.*obj.factors(idx).factor;
                        val1 = sum(sum(T,2),3);
                        val2 = sum(sum(T,1),3);
                        val3 = sum(sum(T,1),2);         
                        val1 = val1./sum(val1);
                        val2 = val2./sum(val2);
                        val3 = val3./sum(val3);
                        val = [val1(:),val2(:),val3(:)];
                                                 
                    otherwise
                        error('suports only 2 or 3 way factors');
                end
                                
                % dims = 1:obj.dim;
                % M = ones(obj.dim,1);
                % for i=1:d
                %     dim_i = circshift(dims,i);
                %     v = permute(obj.keys(idx2(i)).value(:),dim_i);
                %     M = M.*v;
                % end                
                % res = M.*T;
                % 
                % val = zeros(obj.dim,d);
                % 
                % a=[1,0]; b=[1,0]; T=[0,1;1,0]/.2;
                % ab=a'*b; res = ab.*T; ares = sum(res,2); bres=sum(res,1)';
                % ares,bres

                % % A : (a x c x Z)
                % % B : (c x b x Z)
                % Ap = permute(A,[2,1,4,3]); % (c x a x 1 x Z)
                % Bp = permute(B,[1,4,2,3]); % (c x 1 x b x Z)
                % M = Ap .* Bp;              % (c x a x b x Z)
                % M = sum(M,1);              % (1 x a x b x Z)
                % M = permute(M,[2,3,4,1]);  % (a x b x Z)
                
            end
        end
        
        
        %------------------------------------------------------------------
        function [res] = calc_composition_level(obj, method, debug, ax)
            %% calc composition level for each node in the graph
            % method - "normlized" 
            
            res=true;
            
            if nargin<3
               debug=false; 
            end
            if nargin<4
                ax=[];
            end
            if debug && isempty(ax)
                figure('name','factor grapg propagation');
                ax=gca;
            end
            
            d = obj.dim;
            n_nodes = numel(obj.keys);
            n_factors = numel(obj.factors);

            nodeids = [obj.keys.id];
            marginals = zeros(d, n_nodes);

            switch method
                
                case "normlized"
                    % approximated algorithm:
                    % 1) mark oracles as composition level as their information score
                    %
                    % 2) calc all updatable factors:
                    %    for each factors that have updated + non updated
                    %    nodes, propagate information score for non updated
                    %    nodes (but don't update yet)
                    %
                    % 3) update most informative factor:
                    %
                    % 4) continue updates untill all nodes are updated or 
                    %    no more updatable nodes 
                    
                    decay_factor = 0.75;
                    
                    % mark oracle nodes as updated
                    prev_is_updated = [];
                    propagation_step = 0;
                    is_updated = [obj.keys.is_oracle];
                    for i=1:numel(obj.keys)
                        obj.keys(i).composition_level =  obj.information_score(obj.keys(i).value);
                    end

                    if debug
                        [plt,~, xy_nodes, xy_factors]=obj.plot(ax,'composition_level');
                        hold(ax,'on');
                        x = xy_nodes(:,1);
                        y = xy_nodes(:,2);
                        p1=plot(ax,x(is_updated==1),y(is_updated==1),'o','MarkerEdgeColor',[0,0.8,0],'MarkerFaceColor','none','MarkerSize',14);
                        title(ax, sprintf('propagate step 0\n'));
                        hold(ax,'off');
                        p2=[];
                    end
                    
                    % stop when all nodes are updated, or no new nodes updated in last step
                    is_key_calculated = zeros(n_nodes,1);
                    key_calculated_value = nan(n_nodes,1);
                    key_calculated_score = nan(n_nodes,1);
                    while ~all(is_updated) && ~isequal(is_updated,prev_is_updated)  % loop untill all nodes are updated, or untill no change
                        prev_is_updated = is_updated;
                        
                        % find which nodes are already updated
                        updated_node_ids = nodeids(is_updated);
                        nonupdated_node_ids = nodeids(~is_updated);
                        
                        % calculate all updatable factors (factors with updated and non-updated nodes)
                        %    calc marginal for npn-updated factors that are 
                        %    connected to an updatable factor
                        for i=1:numel([obj.factors])
                            [C1, ~, ~] = intersect(obj.factors(i).keys, nonupdated_node_ids);
                            [C2, ~, ~] = intersect(obj.factors(i).keys, updated_node_ids);
                            if numel(C1)==1   % one nonupdated node in the factor
                                idx0 = nodeids==C1;
                                idx1 = nodeids==C2(1);
                                idx2 = nodeids==C2(2);
                                % 2->1 propagate
                                c1 = obj.keys(idx1).composition_level;
                                c2 = obj.keys(idx2).composition_level;
                                c = mean([c1,c2])*decay_factor;
                                if ~is_key_calculated(idx0) || (is_key_calculated(idx0) && c > key_calculated_score(idx0))
                                    is_key_calculated(idx0) = true;
                                    key_calculated_value(idx0) = c;
                                    key_calculated_score(idx0) = c;
                                end
                            end
                            
                            if numel(C1)==2   % two nonupdated nodes in the factor
                                unobserved_node_idx1 = nodeids==C1(1);
                                unobserved_node_idx2 = nodeids==C1(2);
                                observed_node_idx = nodeids==C2;
                                
                                % 1 -> 2 propagate
                                c0 = obj.keys(observed_node_idx).composition_level;
                                c1 = c0 * (1-sqrt(1-decay_factor));
                                c2 = c0 * (1-sqrt(1-decay_factor));
                                
                                if ~is_key_calculated(unobserved_node_idx1) || (is_key_calculated(unobserved_node_idx1) && c1 > key_calculated_score(unobserved_node_idx1))
                                    is_key_calculated(unobserved_node_idx1) = true;
                                    key_calculated_value(unobserved_node_idx1) = c1;
                                    key_calculated_score(unobserved_node_idx1) = c1;
                                end
                                if ~is_key_calculated(unobserved_node_idx2) || (is_key_calculated(unobserved_node_idx2) && c2 > key_calculated_score(unobserved_node_idx2))
                                    is_key_calculated(unobserved_node_idx2) = true;
                                    key_calculated_value(unobserved_node_idx2) = c2;
                                    key_calculated_score(unobserved_node_idx2) = c2;
                                end
                            end
                        end
                        
                        % update most informative non-updated node (best score)
                        [max_score,idx_max] = max(key_calculated_score);
                        if ~isnan(max_score)
                            obj.keys(idx_max).composition_level = key_calculated_value(idx_max);
                            is_updated(idx_max) = true;
                            
                            % clear connected calculated values
                            % (keep unused calculations for later use)
                            is_key_calculated(idx_max) = false;
                            max_key_id = nodeids(idx_max);
                            connected_factor_ids = obj.keys(idx_max).connected_factors;  % get all connected factors
                            for fi = connected_factor_ids
                                fidx = [obj.factors.id]==fi;
                                [isa, locb] = ismember(nodeids, obj.factors(fidx).keys);  % get all connected keys
                                connected_key_ids = nodeids(isa);
                                for ki = connected_key_ids
                                    idx_tmp = nodeids == ki;
                                    is_key_calculated(idx_tmp) = false;
                                    key_calculated_value(idx_tmp) = nan;
                                    key_calculated_score(idx_tmp) = nan;
                                end
                            end
                        end

                        % plot
                        if debug
                            fprintf('propagating node %d\n',max_key_id);
                            propagation_step = propagation_step + 1;
                            
                            [sum(is_key_calculated), sum(is_updated)]
                            
                            [plt,~, xy_nodes, xy_factors]=obj.plot(ax,'composition_level');
                            hold(ax,'on');
                            x = xy_nodes(:,1);
                            y = xy_nodes(:,2);
                            p1=plot(ax,x(is_key_calculated==1),y(is_key_calculated==1),'o','MarkerEdgeColor','r','MarkerFaceColor','none','MarkerSize',14);
                            p2=plot(ax,x(is_updated==1),y(is_updated==1),'o','MarkerEdgeColor',[0,0.8,0],'MarkerFaceColor','none','MarkerSize',14);
                            title(ax, sprintf('propagate step %d\n',propagation_step));
                            
                            hold(ax,'off');
                        end
                        
                    end
                       
                otherwise
                    error('invalig marginals calculation method!');
                    
            end
            
        end        
        
        
        %------------------------------------------------------------------
        function write(obj, resdir, name, mode)
            % write factor graph to file 
            % resdir - output folder
            % name - common name for result files
            % mode:
            %      GTSAM - complient to GTSAM discrete factor graph C++ script
            %              we transpose the factor potential to comply with GTSAM
            %              we don't use factor id to comply with GTSAM
            %              we use a single common factor value (this is 
            %              only script limitations, to GTSAM discrete)
            %              this factor potential is written once in a 
            %              seperate "factor_potential" file
            %      normal - more general and intuitive with this class
            
            if nargin < 4
               mode = 'normal'; 
            end
            if nargin <3
                name = '';
            end
            if nargin <2
                resdir = '';
            end
            
            if ~isempty(resdir) && ~exist(resdir,'dir')
                mkdir(resdir)
           end
           
           % write priors
           priors_file_name = fullfile(resdir,sprintf('dfg_priors%s.txt', name));
           priors_file = fopen(priors_file_name,'w');
           for i = 1:numel(obj.keys())
                fprintf(priors_file, '%d ',obj.keys(i).id);
                fprintf(priors_file, '%f ',obj.keys(i).value);
                fprintf(priors_file, '\n');
           end
           fclose(priors_file);
           
           % write priors
           marginals_file_name = fullfile(resdir,sprintf('dfg_marginals%s.txt', name));
           marginals_file = fopen(marginals_file_name,'w');
           for i = 1:numel(obj.keys())
               if ~isempty(obj.keys(i).marginal)
                   fprintf(marginals_file, '%d ',obj.keys(i).id);
                   fprintf(marginals_file, '%f ',obj.keys(i).marginal);
                   fprintf(marginals_file, '\n');
               end
           end
           fclose(marginals_file);           
           
           % write factors
           switch mode
               case 'normal'
                   factors_file_name = fullfile(resdir,sprintf('dfg_factors%s.txt', name));
                   factors_file = fopen(factors_file_name,'w');
                   for i = 1:numel(obj.factors)
                        fprintf(factors_file, '%d ',obj.factors(i).id);
                        fprintf(factors_file, '%d ',obj.factors(i).keys);
                        n = numel(obj.factors(i).keys);
                        fv = permute(obj.factors(i).factor,n:-1:1);
                        fprintf(factors_file, '%f ',fv(:));
                        fprintf(factors_file, '\n');
                   end
                   fclose(factors_file);
                   
               case 'GTSAM'
                   factors_file_name = fullfile(resdir,sprintf('dfg_factors_potential%s.txt', name));
                   factors_potential_file = fopen(factors_file_name,'w');
                   
                   factors_file_name = fullfile(resdir,sprintf('dfg_factors%s.txt', name));
                   factors_file = fopen(factors_file_name,'w');
                   
                   p = [];
                   for i = 1:numel(obj.factors)
                       if isempty(p)
                           p = obj.factors(i).factor;
                           n = numel(obj.factors(i).keys);
                           fv = permute(p,n:-1:1);
                           fprintf(factors_potential_file, '%f ',fv(:));
                       else
                           if ~isequal(p, obj.factors(i).factor) || ~isequal(n, numel(obj.factors(i).keys))
                               error('on GTSAM mode all factors must be the same!');
                           end
                       end
                       fprintf(factors_file, '%d ',obj.factors(i).id);
                       fprintf(factors_file, '%d ',obj.factors(i).keys);
                       fprintf(factors_file, '\n');
                   end
                   fclose(factors_potential_file);
                   fclose(factors_file);
                   
               otherwise
                   error('invalid write mode!');
           end
           
        end 
     
        function load(obj, priors_file, marginals_file, factors_file, factor_potential_file, mode)
            % load factor graph from file (form GTSAM)
            % - priors_file - value for each node
            % - marginals_file - marginal value for each node
            % - factors_file - factors
            %                2 possible formats:
            %                1) node ids + factor potential (normal mode)
            %                2) node ids only (GTSAM mode)
            % - factor_potential_file - common potental for all factors 
            %                           (only in GTSAM mode) 
            % - mode:
            %      GTSAM - complient to GTSAM discrete factor graph C++ script
            %              we transpose the factor potential to comply with GTSAM
            %              we don't use factor id to comply with GTSAM
            %              we use a single common factor value (this is 
            %              only script limitations, to GTSAM discrete)
            %              this factor potential is written once in a 
            %              seperate "factor_potential" file
            %      normal - more general and intuitive with this class
            
            % check inputs
            if nargin <4
                error('invalid input');
            end
            if ~exist(priors_file,'file')
                error('file not found: %s',priors_file);
            end
            if ~isempty(marginals_file) && ~exist(marginals_file,'file')
                error('file not found: %s',marginals_file);
            end
            if ~exist(factors_file,'file')
                error('file not found: %s',factors_file);
            end
            if nargin<6
                mode = 'normal';
            end
            if nargin>=5
                if strcmp(mode,'GTSAM') && ~exist(factor_potential_file,'file')
                    error('file not found: %s',factor_potential_file);
                end
            end
            
            % read priors
            data = dlmread(priors_file);
            d = size(data,2)-1;
            if d~=obj.dim
                error('invalid dim!');
            end
            for i=1:size(data,1)
                obj.addKey(data(i,1),data(i,2:end));
            end
            
            % read marginals
            if ~isempty(marginals_file)
                data = dlmread(marginals_file);
                d = size(data,2)-1;
                if d~=obj.dim
                    error('invalid dim!');
                end
                nodeids = [obj.keys.id];
                for i=1:size(data,1)
                    idx = nodeids==data(i,1);
                    obj.keys(idx).marginal = data(i,2:end);
                end
            end
            
            % read factors
            switch mode
                case 'normal'
                    fid = fopen(factors_file); % open the file
                    while ~feof(fid) % feof(fid) is true when the file ends
                        textLineEntry = fgetl(fid); % read one line
                        data = sscanf(textLineEntry,'%f');
                        sp = numel(data);
                        switch sp
                            case 1+1+obj.dim
                                n=1;
                            case 1+2+obj.dim^2
                                n=2;
                            case 1+3+obj.dim^3
                                n=3;
                            otherwise
                                error('factors that are more than trinary are not yet supported');
                        end
                        factorid = data(1);
                        keyids = data(2:n+1);
                        fp = permute(reshape(data(n+2:end),obj.dim*ones(1,n)),n:-1:1);
                        obj.addFactor(factorid,keyids,fp);
                    end
                    fclose(fid); % close the file
                    
                case 'GTSAM'
                    fp = dlmread(factor_potential_file);
                    n = log(numel(fp(:)))/log(obj.dim);
                    fp = permute(reshape(fp,obj.dim*ones(1,n)),n:-1:1);
                    
                    data = dlmread(factors_file);
                    if size(data,2)~=n+1
                       error('invalid input: %f keys, key dim=%f, factor potiential size=%f',size(data,2),obj.dim,numel(fp(:))); 
                    end
                    for i=1:size(data,1)
                        obj.addFactor(data(i,:),data(i,2:end),fp);
                    end
                    
                otherwise
                    error('invalid load mode!');
            end
        end
        
        %------------------------------------------------------------------
        function [p,ax, xy_nodes, xy_factors]=plot(obj,ax,mode)
            %% plot
            % f - figure
            % mode: 
            %    - value - text node value
            %    - marginal - text node marginal
            % node_ids: plot only specific node ids, and all
            %           interconnecting factors
            
            if nargin<3
                mode = 'value';
            end
            n = numel(obj.keys);
            m = numel(obj.factors);
            if n>100
               wraning('too many tracks to plot!'); 
               return;
            end
            if nargin==1
                figure('name','observability graph');
                ax=gca;
            end
                        
            nodenames=cell(n+m,1);
            nodelabels=cell(n+m,1);
            nodecolors=zeros(n+m,1);
            nodemarkers=cell(n+m,1);
            nodemarkersize=zeros(n+m,1);
            for i=1:n
                nodenames{i} = num2str(obj.keys(i).id);
                switch mode
                    case 'marginal'
                        v = obj.information_score(obj.keys(i).marginal);
                    case 'value'
                        v = obj.information_score(obj.keys(i).value);
                    case 'composition_level'
                        v = obj.keys(i).composition_level;
                    otherwise
                        error('invalid mode!');
                end
                
                nodelabels{i} = sprintf('%d(%.2f)',obj.keys(i).id, v);
                nodecolors(i,:) = 1;
                nodemarkers{i} = 'o';
                nodemarkersize(i) = 7;
            end
            for i=n+1:n+m
                nodenames{i} = sprintf('f:%d',obj.factors(i-n).id);                
                nodelabels{i} = sprintf('f:%d',obj.factors(i-n).id);
                nodecolors(i,:) = 2;
                nodemarkers{i} = '^';
                nodemarkersize(i) = 9;
            end
            
%             [G2, idx] = reordernodes(obj.G, nodenames);
%             nodecolors = nodecolors(idx);
%             nodelabels = nodelabels(idx);
%             nodemarkers = nodemarkers(idx);
%             nodemarkersize = nodemarkersize(idx);
            
            p=obj.G.plot('Parent',ax,'NodeCData',nodecolors,...
                'NodeLabel',nodelabels,...
                'Marker',nodemarkers,...
                'MarkerSize',nodemarkersize);  
            
            xy_nodes = [p.XData(1:n)',p.YData(1:n)'];
            xy_factors = [p.XData(n+1:n+m)',p.YData(n+1:n+m)'];
%             colorbar;
        
        end
        
    end
    
end