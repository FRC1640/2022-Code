package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Jetson {

    private final int UDP_PORT = 3000;
    private VisionTargets targets = new VisionTargets();
    private DatagramSocket socket;
    private byte[] buffer;
    private ObjectMapper mapper;
    private static Alliance allianceColor;

    private static Jetson instance = new Jetson();

    private Jetson() {
        buffer = new byte[256];
        mapper = new ObjectMapper();

        try {
            socket = new DatagramSocket(UDP_PORT);

            new Thread(() -> this.execute()).start();
        } catch (SocketException e) {
            e.printStackTrace();
        }

    }

    public void execute() {

        while(true) {
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            try {
                socket.receive(packet);
                String json = new String(packet.getData(), 0, packet.getLength());
                List<JetsonWrapper> jetsonList = mapper.readValue(json, new TypeReference<List<JetsonWrapper>>(){}).stream().sorted().collect(Collectors.toList());
                targets = new VisionTargets();

                // for(JetsonWrapper wrapper : jetsonList) {
                //     if("1".equals(wrapper.getColor())) {
                //         targets.getList1().add(wrapper);
                //     }
                //     else if("2".equals(wrapper.getColor())) {
                //         targets.getList2().add(wrapper);
                //     }
                // }

                //System.out.println(jetsonList);
            } catch (IOException e) {
                e.printStackTrace();
            }

        }

    }

    public static String getFMSColor() {
        allianceColor = DriverStation.getAlliance();

        if(allianceColor.equals(Alliance.Blue)) {
            return "2";
        }

        else if(allianceColor.equals(Alliance.Red)) {
            return "1";
        } else {
            //default to red
            return "1";
        }
    }

    public static JetsonWrapper get() {

        // if(instance.targets.getList1().size() > 0) {
        //     return instance.targets.getList1().get(0);
        // }

        return null;
    }
    
}
