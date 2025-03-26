package org.rawrobotics.common.components;

import org.rawrobotics.common.annotations.Robot;
import org.rawrobotics.common.interfaces.IRobotContainer;
import lombok.experimental.UtilityClass;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.jar.JarEntry;
import java.util.jar.JarInputStream;

@UtilityClass
public class RobotContainerRegistry {

    // A map to hold robot containers by team
    private static final Map<Integer, Class<?>> teamContainers = new HashMap<>();

    static {
        // Register the containers for each team
        for (Class<?> clazz : getAllClasses()) {
            if (clazz.isAnnotationPresent(Robot.class)) {
                Robot annotation = clazz.getAnnotation(Robot.class);
                teamContainers.put(annotation.team(), clazz);
            }
        }
    }

    public static IRobotContainer createContainerForTeam(int teamNumber) {
        // Try to get the container for the specific team
        Class<?> containerClass = teamContainers.get(teamNumber);

        // If not found, use the default container
        if (containerClass == null) {
            throw new RuntimeException("No Robot Containers!");        
        }

        try { //Create the container
            return (IRobotContainer) containerClass.getMethod("createContainer").invoke(null);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return null; 
    }

    private static final String PACKAGE_NAME = "frc.robot";  

    private static List<Class<?>> getAllClasses() {
        List<Class<?>> classes = new ArrayList<>();
        String path = PACKAGE_NAME.replace('.', '/');
        
        try (InputStream is = RobotContainerRegistry.class.getClassLoader()
                .getResourceAsStream(path);
            JarInputStream jarStream = new JarInputStream(is)) {
            
            JarEntry entry;
            while ((entry = jarStream.getNextJarEntry()) != null) {
                if (entry.getName().endsWith(".class") && entry.getName().startsWith(path)) {
                    String className = entry.getName()
                            .replace('/', '.')
                            .replace(".class", "");
                    Class<?> clazz = Class.forName(className);
                    classes.add(clazz);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return classes;
    }
}

