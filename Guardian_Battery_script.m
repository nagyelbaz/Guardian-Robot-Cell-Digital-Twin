close all
clc

%% Current and time

driveCycle = readtable('Guardian_Battery_DriveCycle_Current (1)');
ocvTable   = readtable('Guardian_Battery_OCV_SOC_Table (1)');

Time_s    = driveCycle.Time_s;
Current_A = driveCycle.Current_A;

soc_bp = ocvTable.SOC_pct;
ocv_bp = ocvTable.OCV_V;

Current = [Time_s , Current_A];

End = max(Time_s);

%% Constants

Ts = 1;
Q_nominal_capacity = 50; % Ah
R0 = 0.0007; % ohm
R1 = 0.0016; % ohm
C1 = 17000; % F
R2 = 0.00035; % ohm
C2 = 5200; % F
taw1 = R1 * C1;
taw2 = R2 * C2;
a1 = exp(-Ts/(taw1));
a2 = exp(-Ts/(taw2));

%% Bus

elems(1) = Simulink.BusElement;
elems(1).Name = 'SOC_CC'; elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'SOC_Flag'; elems(2).DataType = 'boolean';

elems(3) = Simulink.BusElement;
elems(3).Name = 'SOC_hat'; elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'V1_hat'; elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'R0_TLS'; elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'SOH_ratio'; elems(6).DataType = 'double';

SensorDataBus = Simulink.Bus;
SensorDataBus.Elements = elems;

%% Plots

subplot( 4 , 4 , [1 5 9 13])
plot(Time_s , out.OCV_used.signals.values , Color = "b")
xlabel('Time (s)')
ylabel('OCV USED (V)')
grid on

subplot( 4 , 4 , [2 6 10 14])
plot(Time_s , out.V1.signals.values , Color = "g")
xlabel('Time (s)')
ylabel('V1 (V)')
grid on

subplot( 4 , 4 , [3 7 11 15])
plot(Time_s , out.SOC_CC.signals.values , Color = "r")
xlabel('Time (s)')
ylabel('SOC CC (%)')
grid on

subplot( 4 , 4 , [4 8 12 16])
plot(Time_s , out.Terminal_Voltage.signals.values , Color = "c")
xlabel('Time (s)')
ylabel('Voltage TERMINAL (V)')
grid on
