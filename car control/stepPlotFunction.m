

function h = stepPlotFunction(agentsList,hist,plot_handles,i,pd,varargin)

if nargin == 6
    plotCar = @(p,theta)varargin{1}.plot(p,theta);
else
    plotCar = @(p,theta)agentsList{1}.plot(p,theta);
end
    
if not(plot_handles == 0)
    delete(plot_handles)
end

t = hist{1}.time(:,1:i);
pdt = cell2mat(arrayfun(pd,t,'UniformOutput',0));
hold off
plot(pdt(1,:),pdt(2,:),'.-');
hold on

x = hist{1}.measurements(:,1:i); hold on;
x(x>20)=nan(size(x(x>20)));


h(1) = plot(x(1,:),x(2,:),'o--');

if agentsList{1}.ny == 3
for j=1:10:size(x,2)
    plotCar(x(1:2,j),x(3,j));
end
end
if isa(agentsList{1}.controller,'MpcController') && ~isempty(agentsList{1}.controller.lastSolution)
x_opt = agentsList{1}.controller.lastSolution.x_opt;
%iOpt  = agentsList{1}.controller.mpcOpSolver.iLastSolution;
    plot(x_opt(1,:),x_opt(2,:),'r-.');
%    plot(x_opt(1,iOpt),x_opt(2,iOpt),'ro');
end

    

end
