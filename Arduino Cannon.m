% Configure pins (STAGE 1)

configurePin(a,'D2','DigitalInput');
configurePin(a,'D3','DigitalInput');
configurePin(a,'D4','DigitalInput');
configurePin(a,'D5','DigitalInput');
configurePin(a,'A0','AnalogInput');
configurePin(a,'A1','AnalogInput');





% Array listing the button pin numbers

buttonPins = [2 3 4 5];

% Array initializing the button states

buttons = [0 0 0 0];


% simulation parameters
xInit = 0;              % initial x position (m)
yInit = 0;              % initial y position (m)
vInit = 25;             % muzzle velocity (m/s)
dt = 0.025;             % time-step (s)
kDrag = 0.03;           % drag parameter
m = 1;                  % ball mass (kg)
g = 9.81;               % gravity  (m/s^2)

vWind = 0;              % initial value

% plotting parameters
xPlotMin = -20;         % lower limit for plotting in x
xPlotMax = 50;          % upper limit for plotting in x
yPlotMin = 0;           % lower limit for plotting in y
yPlotMax = 20;          % upper limit for plotting in y

% run indefinitely (infinite loop)
while true    
    
    % get button states (STAGE 1
    
    for i = 1:4
        buttonStr = sprintf('D%d', buttonPins(i));
        buttons(i) = readDigitalPin (a,buttonStr);
    end
   
    
    
   % print buttons (STAGE 1)
   
   disp (['Buttons: ' num2str(buttons)]);
   
   % Exit the porogram if more than one button is pressed simultaneously
   
   if sum(buttons) < 3
       break;
   end
   
    
   
   
    % update parameters (STAGE 2)
    
    pot1 = readVoltage(a,'A0');
    pot2 = readVoltage(a,'A1');
    
    
    % update angle
    angle = (pot1 / 5 * 80) + 10;
    
    %update wind
    
    if pot2 > 2.5
        vWind = ((pot2 - 2.5) * 4);
    elseif pot2 < 2.5
        vWind = -10 + (pot2 * 4);
    else 
        vWind = 0;
    end
    
    
    % print buttons (STAGE 1), angle, and wind velocity (STAGE 2)
  
    disp (['Buttons: ' num2str(buttons) '| Angle = ' sprintf('%d',angle) '| Wind: ' sprintf('%1.3f m/s',vWind)]);
   
    % exit if more than one button pressed at once (STAGE 1)
    
    if sum(buttons) < 3
       break;
   end
    
    
    
    % clear figure and plot cannon and wind vector
    clf
    hold on
    plot([xPlotMin xPlotMax],[0 0],'color',[0 0.65 0],'linewidth',2);       % ground
    plot([0 cosd(angle)],[0 sind(angle)],'k','linewidth',3);                % barrel
    quiver(0,yPlotMax/2,vWind,0,1,'b','linewidth',2,'MaxHeadSize',0.5);     % wind arrow
    axis equal                                                              % ensure plotting is scaled in both axes
    xlim([xPlotMin xPlotMax]);                                              % set plot limits in x
    ylim([yPlotMin yPlotMax]);                                              % set plot limits in y
    set(gca,'color',[0.75 1 1]);                                            % set sky colour
    getframe;
    
    % fire cannon if any button pressed (run simulation) (STAGE 3)
    if sum(buttons) == 3
       
        % initialize parameters
        
        x = xInit + cosd(angle);
        y = yInit + sind(angle);
        vx = vInit * cosd(angle);
        vy = vInit * sind(angle);
        
        xprev = x;
        yprev = y;
        vx_prev = vx;
        vy_prev = vy;
        
      

        % keep running simulation while ball above ground
        while (yprev >= 0)
            
         
            % forces:
            
            Fgrav = -m*g;
            F_dragx = -kDrag *(vx - vWind)^2 * sign(vx - vWind);
            F_dragy = -kDrag * vy^2 * sign(vy);
            F_netx = F_dragx;
            F_nety =  F_dragy + Fgrav;
            
            
            
            

            % accelerations:
            
            ax = (F_netx)/(m);
            ay = (F_nety)/(m);
            

            % update velocities:
           
            vx = vx_prev + (ax * dt);
            vy = vy_prev + (ay * dt);
            
            

            % get average velocities:]
            
            vx_ave = 0.5 * (vx + vx_prev);
            vy_ave = 0.5 * (vy + vy_prev);
            

            % update positions:
            
            x = xprev + (vx_ave * dt);
            y = yprev + (vy_ave * dt);

            % update previous values:
            
            xprev = x;
            yprev = y;
            vx_prev = vx;
            vy_prev = vy;
            
            
            % plot ball (additive):
            
            plot (x,y ,'r.');
            
            getframe;
            
            % redraw wind...
            
            
        end
    end
    
end

% close figure window
close(gcf);