package net.comsoria.controller;

import net.comsoria.engine.libraryImplementation.AudioLibrary;
import net.comsoria.engine.Scene;
import net.comsoria.engine.audio.AudioBuffer;
import net.comsoria.engine.audio.AudioNode;
import net.comsoria.engine.audio.AudioManager;
import net.comsoria.engine.audio.AudioSource;
import net.comsoria.engine.loaders.FileLoader;
import net.comsoria.engine.loaders.xhtml.XHTMLLoader;
import net.comsoria.engine.loaders.xhtml.ui.Document;
import net.comsoria.engine.loaders.xhtml.ui.DocumentHandler;
import net.comsoria.engine.loaders.xhtml.ui.UINode;
import net.comsoria.engine.loaders.xhtml.ui.node.Canvas;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.FadeFog;
import net.comsoria.engine.view.FrameBuffer;
import net.comsoria.engine.view.light.DirectionalLight;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.color.Color3;
import net.comsoria.engine.view.graph.Texture;
import net.comsoria.engine.view.input.KeyListener;
import net.comsoria.game.Player;
import net.comsoria.game.SkyDome;
import net.comsoria.game.terrain.terrainFeature.Octave;
import net.comsoria.game.terrain.terrainFeature.cave.cave.CaveLoader;
import net.comsoria.game.terrain.terrainFeature.cave.cave.generation.CaveOctaveGenerator;
import net.comsoria.game.terrain.terrainFeature.surfaceChunk.SurfaceChunkLoader;
import net.comsoria.game.terrain.World;

import net.comsoria.game.terrain.terrainFeature.surfaceChunk.generation.SurfaceChunkOctaveGenerator;
import org.joml.Vector2d;
import org.joml.Vector3f;
import org.joml.Vector3i;

import java.util.Arrays;

import static org.lwjgl.glfw.GLFW.*;

public class GameHandler extends DocumentHandler {
    private World world;
    private Player player;

    private boolean paused = true;
    private float time;

    private Scene scene;

    private UINode compass;

    private final AudioManager soundManager = new AudioManager();
    private AudioSource sourceBack;

    @Override public Document init(Window window) throws Exception {
        Document document = XHTMLLoader.loadDocument(FileLoader.loadResourceAsStringFromPath("$uis/game.xhtml"), window);
        document.updateAllStyleSets(window);

        document.frameBufferRenderer.frameBuffers.add(new FrameBuffer(
                window.getWidth(), window.getHeight(),
                FileLoader.loadResourceAsStringFromPath("$shaders/post_processing/post_processing.v.glsl"),
                FileLoader.loadResourceAsStringFromPath("$shaders/post_processing/post_processing.f.glsl"))
        );

        scene = ((Canvas) document.getElementByID("main")).scene;
        scene.camera.far = 6000;

        compass = document.getElementByID("compass");
        compass.getMesh().rotation.z = 180;

        player = new Player(new Vector3f(0, 0, 0));
        world = new World(4000);
        scene.add(world);

        float skyDomeR = scene.camera.far - 100;
        scene.add(SkyDome.genSkyDome(
                FileLoader.loadResourceAsStringFromPath("$shaders/skydome/skydome.v.glsl"),
                FileLoader.loadResourceAsStringFromPath("$shaders/skydome/skydome.f.glsl"),
                skyDomeR, new Texture(Utils.utils.getPath("$textures/sun.png"))
        ));

        SurfaceChunkOctaveGenerator generator = new SurfaceChunkOctaveGenerator(Arrays.asList(
                new Octave(0.05f, 0.06f, (float) Math.random() * 100),
                new Octave(0.025f, 0.12f, (float) Math.random() * 100),
                new Octave(0.015f, 0.25f, (float) Math.random() * 100)
        ));
        generator.overallHeight = 0.7f;

        world.addLoader(new SurfaceChunkLoader(generator, 65, skyDomeR));
        world.addLoader(new CaveLoader(
                skyDomeR,
                (float) Math.random() * 100,
                (float) Math.random() * 100,
                (float) Math.random() * 100,
                2,
                new CaveOctaveGenerator(Arrays.asList(
                        new Octave(0.05f, 5f, (float) Math.random() * 100),
                        new Octave(0.1f, 2f, (float) Math.random() * 100),
                        new Octave(0.2f, 1f, (float) Math.random() * 100)
                )),
                100,
                0.01f
        ));
//        chunkLoader.updateAroundPlayer(player.get2DPosition(), world);
        world.updateAroundPlayer(player.get2DPosition());

        keyInput.addListener(new KeyListener(new int[]{GLFW_KEY_ESCAPE}, (charCode, action) -> {
            if (action != GLFW_RELEASE) return;
            paused = !paused;
            if (paused) {
                window.showCursor();
                window.setMousePos((double) window.getWidth() * 0.25, (double) window.getHeight() * 0.25);
                sourceBack.pause();
            } else {
                sourceBack.play();
                window.hideCursor();
            }
        }, false));

        Color3 background = new Color3(23, 32, 42).getOneToZero();
        window.setClearColor(background);
        scene.fog = new FadeFog(0.0009f, skyDomeR - 1500);

        scene.light.directionalLight = new DirectionalLight(new Color3(250, 215, 160).getOneToZero(), new Vector3f(), 0.55f);

//        scene.add(chunkLoader.batchRenderer);

        soundManager.init();
        soundManager.setAttenuationModel(AudioLibrary.Model.Exponent);

        AudioBuffer buffBack = new AudioBuffer(Utils.utils.p("$audio/bg.ogg"));
        soundManager.addSoundBuffer(buffBack);
        sourceBack = new AudioSource(true, true);
        sourceBack.setBuffer(buffBack.getBufferId());
        sourceBack.setGain(0.5f);

        soundManager.addSoundSource("BACKING", sourceBack);

        soundManager.setListener(new AudioNode(new Vector3f(0, 0, 0)));

        return document;
    }

    @Override
    public DocumentHandler update(Window window, Document document, float interval) throws Exception {
        if (window.isResized()) {
            document.updateAllStyleSets(window);
        }

        if (!paused) {
            Vector3i movement = new Vector3i();
            if (keyInput.isKeyPressed(GLFW_KEY_W)) {
                movement.z -= 1;
            }

            if (keyInput.isKeyPressed(GLFW_KEY_S)) {
                movement.z += 1;
            }

            if (keyInput.isKeyPressed(GLFW_KEY_A)) {
                movement.x -= 1;
            }

            if (keyInput.isKeyPressed(GLFW_KEY_D)) {
                movement.x += 1;
            }

            if (keyInput.isKeyPressed(GLFW_KEY_SPACE)) {
                movement.y += 1;
            }

            if (keyInput.isKeyPressed(GLFW_KEY_LEFT_SHIFT)) {
                movement.y -= 1;
            }

            float speed = player.getSpeed(keyInput.isKeyPressed(GLFW_KEY_LEFT_CONTROL));
            scene.camera.movePosition((movement.x / 15f) * speed, ((movement.y / 15f) * speed), ((movement.z / 15f) * speed));

            Vector2d pos = mouseInput.getMovementVec();
            scene.camera.rotation.x += (float) pos.y * 0.07f;
            scene.camera.rotation.y += (float) pos.x * 0.07f;

            soundManager.updateListenerPosition(scene.camera);

            compass.getMesh().rotation.z = scene.camera.rotation.y + 180f;

            player.setPosition(scene.camera.position);

            if (keyInput.isKeyPressed(GLFW_KEY_UP)) {
                time += 0.1f;
            }
            if (keyInput.isKeyPressed(GLFW_KEY_DOWN)) {
                time -= 0.1f;
            }
        }

//        time += 0.005;
        scene.updateLight(time);

        world.updateAroundPlayer(player.get2DPosition());
        return null;
    }

    @Override public void cleanup() {
        sourceBack.stop();
        soundManager.cleanup();
    }
}
package net.comsoria.controller;

import net.comsoria.engine.loaders.FileLoader;
import net.comsoria.engine.loaders.xhtml.XHTMLLoader;
import net.comsoria.engine.loaders.xhtml.ui.Document;
import net.comsoria.engine.loaders.xhtml.ui.DocumentHandler;
import net.comsoria.engine.view.Window;

public class HomeHandler extends DocumentHandler {
    @Override public Document init(Window window) throws Exception {
        Document document = XHTMLLoader.loadDocument(FileLoader.loadResourceAsStringFromPath("$uis/home.xhtml"), window);
        document.updateAllStyleSets(window);
        return document;
    }

    @Override public void cleanup() {

    }

    @Override
    public DocumentHandler update(Window window, Document document, float interval) throws Exception {
        if (window.isResized()) document.updateAllStyleSets(window);
        return null;
    }
}
package net.comsoria.controller;

import net.comsoria.engine.loaders.FileLoader;
import net.comsoria.engine.loaders.xhtml.XHTMLLoader;
import net.comsoria.engine.loaders.xhtml.ui.Document;
import net.comsoria.engine.loaders.xhtml.ui.DocumentHandler;
import net.comsoria.engine.loaders.xhtml.ui.StyleSet;
import net.comsoria.engine.loaders.xhtml.ui.UINode;
import net.comsoria.engine.utils.Timer;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.color.Color4;

public class LoadingHandler extends DocumentHandler {
    private UINode center;

    private final static double wait = 50, fade = 50;

    @Override public Document init(Window window) throws Exception {
        Document document = XHTMLLoader.loadDocument(FileLoader.loadResourceAsStringFromPath("$uis/loading.xhtml"), window);
        document.updateAllStyleSets(window);

        this.center = document.getElementByID("center");

        return document;
    }

    @Override public void cleanup() {

    }

    @Override public DocumentHandler update(Window window, Document document, float interval) throws Exception {
        if (window.isResized()) {
//            StyleSet.StyleRule rule = this.center.styleSet.ruleMap.get("scale");
//            rule.setValue(Float.valueOf(rule.value) * (window.getWidth() / 800f));

            document.updateAllStyleSets(window);
        }

        float opacity = (1 - (float) Math.max((Timer.getGameLoopIndex() - wait) / fade, 0)) - 0.02f;
        center.styleSet.ruleMap.get("color").setValue(Color4.BLACK.clone().setA(opacity).toString(false));
        center.updateStyleSets(window);

        return Timer.getGameLoopIndex() == wait + fade? new GameHandler():null;
    }
}
package net.comsoria.controller;

import net.comsoria.engine.GameEngine;
import net.comsoria.engine.IGameLogic;
import net.comsoria.engine.utils.Logger;

import java.io.BufferedOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;

public class Main {
    public static final String VERSION = "1.3";

    public static void main(String[] args) throws IOException {
        Logger.log("Started game Cosmoria");
        StartupFileHandler.load();
        GameEngine engine = new GameEngine("Cosmoria", 800, 520, new LoadingHandler(), true);
        engine.start();
    }
}
package net.comsoria.controller;

import net.comsoria.engine.utils.JSONFile;
import net.comsoria.engine.utils.Logger;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.loaders.CSVLoader;
import net.comsoria.engine.loaders.FileLoader;
import net.comsoria.engine.loaders.WebLoader;

import java.io.*;
import java.util.Date;

public class StartupFileHandler {
    public static void load() throws IOException {
        Utils.utils.addName("home", System.getProperty("user.home") + "/Cosmoria");
        Utils.utils.addName("saves", "$home/saves");
        Utils.utils.addName("res", "$home/resources");

        Utils.utils.addName("shaders", "$res/shaders");
        Utils.utils.addName("models", "$res/models");
        Utils.utils.addName("textures", "$res/textures");
        Utils.utils.addName("uis", "$res/UIs");
        Utils.utils.addName("fonts", "$res/fonts");
        Utils.utils.addName("audio", "$res/audio");
        Utils.utils.addName("logs", "$home/logs");

        Utils.utils.createDirs(new String[] {
                "$home", "$saves", "$res", "$models", "$shaders", "$textures", "$uis", "$fonts", "$audio", "$logs"
        });

        File log = new File(Utils.utils.p("$logs/v" + Main.VERSION + ".log"));
        if (!log.exists()) log.createNewFile();

        String old = FileLoader.loadResourceAsString(log);
        PrintStream out = new PrintStream(new BufferedOutputStream(new FileOutputStream(log)), true);
        Logger.addOutputStream(out);
        out.println(old + "\n-----------------------------------------------");
//        System.setOut(out);
//        System.setErr(out);
//        System.out.println(old);
//        System.out.println("-----------------------------------------------");
        Logger.log("Began on new output stream");

        Logger.log("Generating files...");

        File settings = new File(Utils.utils.p("$home/settings.json"));

        Utils.utils.addName("git", "https://raw.githubusercontent.com/Thundernerds/CosmoriaResources/master/", true);
        try {
            Logger.log("Downloading new data...");
            CSVLoader csvFile = new CSVLoader(WebLoader.loadResourceFromNet(Utils.utils.p("$git/gitPath.csv")));

            for (int i = 0; i < csvFile.rows(); i++) {
                CSVLoader.Row row = csvFile.getRow(i);

                String path = Utils.utils.getPath(row.getPart(0));
                String gitPath = row.getPart(1);

                File file = new File(path);
                if (!file.exists()) {
                    if (gitPath.equals("null")) {
                        file.mkdir();
                    } else if (gitPath.endsWith(".png") || gitPath.endsWith(".ttf") || gitPath.endsWith(".ogg")) {
                        WebLoader.copyImageFromNet(Utils.utils.p("$git/" + gitPath), path);
                    } else {
                        FileLoader.writeResource(file, WebLoader.loadResourceFromNet(Utils.utils.p("$git/" + gitPath)));
                    }
                }
            }
        } catch (Exception e) {
            if (!settings.exists()) {
                System.err.println("Failed to start - An internet connection is needed the first time you start the game");
                System.exit(-1);
            }
        }

        Utils.utils.settings = new JSONFile(settings);
        Utils.utils.settings.put("last", new Date().getTime());
        Utils.utils.settings.save();
    }
}
package net.comsoria.engine.audio;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;

import net.comsoria.engine.libraryImplementation.AudioLibrary;
import net.comsoria.engine.loaders.FileLoader;
import org.lwjgl.stb.STBVorbisInfo;
import java.nio.ShortBuffer;
import static org.lwjgl.stb.STBVorbis.*;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import static org.lwjgl.system.MemoryUtil.*;

public class AudioBuffer {
    private final int bufferId;
    private ShortBuffer pcm = null;
    private ByteBuffer vorbis = null;

    public AudioBuffer(String file) throws Exception {
        this.bufferId = AudioLibrary.AL.genBuffer();
        try (STBVorbisInfo info = STBVorbisInfo.malloc()) {
            ShortBuffer pcm = readVorbis(file, 32 * 1024, info);

            // Copy to buffer
            AudioLibrary.AL.bufferData(bufferId, info.channels() == 1 ? AudioLibrary.Format.MONO_16 : AudioLibrary.Format.STEREO_16, pcm, info.sample_rate());
        }
    }

    public int getBufferId() {
        return this.bufferId;
    }

    public void cleanup() {
        AudioLibrary.AL.deleteBuffers(this.bufferId);
        if (pcm != null) {
            MemoryUtil.memFree(pcm);
        }
    }

    private ShortBuffer readVorbis(String resource, int bufferSize, STBVorbisInfo info) throws Exception {
        try (MemoryStack stack = MemoryStack.stackPush()) {
            vorbis = FileLoader.ioResourceToByteBuffer(resource, bufferSize);
            IntBuffer error = stack.mallocInt(1);
            long decoder = stb_vorbis_open_memory(vorbis, error, null);
            if (decoder == NULL) {
                throw new RuntimeException("Failed to open Ogg Vorbis file. Error: " + error.get(0));
            }

            stb_vorbis_get_info(decoder, info);

            int channels = info.channels();

            int lengthSamples = stb_vorbis_stream_length_in_samples(decoder);

            pcm = MemoryUtil.memAllocShort(lengthSamples);

            pcm.limit(stb_vorbis_get_samples_short_interleaved(decoder, channels, pcm) * channels);
            stb_vorbis_close(decoder);

            return pcm;
        }
    }
}package net.comsoria.engine.audio;

import net.comsoria.engine.libraryImplementation.AudioLibrary;
import net.comsoria.engine.view.Camera;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import org.joml.Matrix4f;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.lwjgl.system.MemoryUtil.NULL;

public class AudioManager {
    private long device;
    private long context;
    private AudioNode listener;
    private final List<AudioBuffer> soundBufferList;
    private final Map<String, AudioSource> soundSourceMap;
    private final Matrix4f cameraMatrix;

    public AudioManager() {
        soundBufferList = new ArrayList<>();
        soundSourceMap = new HashMap<>();
        cameraMatrix = new Matrix4f();
    }

    public void init() throws Exception {
        this.device = AudioLibrary.AL.openDevice(null);
        if (device == NULL) {
            throw new IllegalStateException("Failed to open the default OpenAL device.");
        }

        int deviceCaps = AudioLibrary.AL.createCapabilites(device);
        this.context = AudioLibrary.AL.createContext(device, null);
        if (context == NULL) {
            throw new IllegalStateException("Failed to create OpenAL context.");
        }

        AudioLibrary.AL.makeContextCurrent(context);
        AudioLibrary.AL.createCapabilites(deviceCaps);
    }

    public void addSoundSource(String name, AudioSource soundSource) {
        this.soundSourceMap.put(name, soundSource);
    }

    public AudioSource getSoundSource(String name) {
        return this.soundSourceMap.get(name);
    }

    public void playSoundSource(String name) {
        AudioSource soundSource = this.soundSourceMap.get(name);
        if (soundSource != null && !soundSource.isPlaying()) {
            soundSource.play();
        }
    }

    public void removeSoundSource(String name) {
        this.soundSourceMap.remove(name);
    }

    public void addSoundBuffer(AudioBuffer soundBuffer) {
        this.soundBufferList.add(soundBuffer);
    }

    public AudioNode getListener() {
        return this.listener;
    }

    public void setListener(AudioNode listener) {
        this.listener = listener;
    }

    public void updateListenerPosition(Camera camera) {
        // Update camera matrix with camera data
        Transformation.updateGenericViewMatrix(camera.position, camera.rotation, cameraMatrix);

        listener.setPosition(camera.position);
        Vector3f at = new Vector3f();
        cameraMatrix.positiveZ(at).negate();
        Vector3f up = new Vector3f();
        cameraMatrix.positiveY(up);
        listener.setOrientation(at, up);
    }

    public void setAttenuationModel(AudioLibrary.Model model) {
        AudioLibrary.AL.setDistanceModel(model);
    }

    public void cleanup() {
        for (AudioSource soundSource : soundSourceMap.values()) {
            soundSource.cleanup();
        }
        soundSourceMap.clear();
        for (AudioBuffer soundBuffer : soundBufferList) {
            soundBuffer.cleanup();
        }
        soundBufferList.clear();
        if (context != NULL) {
            AudioLibrary.AL.destroyContext(context);
        }
        if (device != NULL) {
            AudioLibrary.AL.closeDevice(device);
        }
    }
}package net.comsoria.engine.audio;

import net.comsoria.engine.libraryImplementation.AudioLibrary;
import org.joml.Vector3f;

public class AudioNode {
    public AudioNode() {
        this(new Vector3f(0, 0, 0));
    }

    public AudioNode(Vector3f position) {
        AudioLibrary.AL.listener(AudioLibrary.Parameter.Position, position.x, position.y, position.z);
        AudioLibrary.AL.listener(AudioLibrary.Parameter.Velocity, 0, 0, 0);
    }

    public void setSpeed(Vector3f speed) {
        AudioLibrary.AL.listener(AudioLibrary.Parameter.Velocity, speed.x, speed.y, speed.z);
    }

    public void setPosition(Vector3f position) {
        AudioLibrary.AL.listener(AudioLibrary.Parameter.Position, position.x, position.y, position.z);
    }

    public void setOrientation(Vector3f at, Vector3f up) {
        float[] data = new float[6];
        data[0] = at.x;
        data[1] = at.y;
        data[2] = at.z;
        data[3] = up.x;
        data[4] = up.y;
        data[5] = up.z;
        AudioLibrary.AL.listener(AudioLibrary.Parameter.Orientation, data);
    }
}package net.comsoria.engine.audio;

import net.comsoria.engine.libraryImplementation.AudioLibrary;
import org.joml.Vector3f;

public class AudioSource {
    private final int sourceId;

    public AudioSource(boolean loop, boolean relative) {
        this.sourceId = AudioLibrary.AL.genSource();

        if (loop) {
            AudioLibrary.AL.setSourceProperty(sourceId, AudioLibrary.SourceParam.Looping, true);
        }
        if (relative) {
            AudioLibrary.AL.setSourceProperty(sourceId, AudioLibrary.SourceParam.Relative, true);
        }
    }

    public void setBuffer(int bufferId) {
        stop();
        AudioLibrary.AL.setSourceProperty(sourceId, AudioLibrary.SourceParam.Buffer, bufferId);
    }

    public void setPosition(Vector3f position) {
        AudioLibrary.AL.setSourceProperty(sourceId, AudioLibrary.SourceParam.Position, position.x, position.y, position.z);
    }

    public void setSpeed(Vector3f speed) {
        AudioLibrary.AL.setSourceProperty(sourceId, AudioLibrary.SourceParam.Velocity, speed.x, speed.y, speed.z);
    }

    public void setGain(float gain) {
        AudioLibrary.AL.setSourceProperty(sourceId, AudioLibrary.SourceParam.Gain, gain);
    }

    public void play() {
        AudioLibrary.AL.play(sourceId);
    }

    public boolean isPlaying() {
        return AudioLibrary.AL.isPlaying(sourceId);
    }

    public void pause() {
        AudioLibrary.AL.pause(sourceId);
    }

    public void stop() {
        AudioLibrary.AL.stop(sourceId);
    }

    public void cleanup() {
        stop();
        AudioLibrary.AL.delete(sourceId);
    }
}package net.comsoria.engine;

import net.comsoria.engine.loaders.xhtml.ui.Document;
import net.comsoria.engine.loaders.xhtml.ui.DocumentHandler;
import net.comsoria.engine.utils.Logger;
import net.comsoria.engine.utils.Timer;
import net.comsoria.engine.view.FrameBuffer;
import net.comsoria.engine.view.FrameBufferRenderer;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.input.KeyInput;
import net.comsoria.engine.view.input.MouseInput;

import java.io.IOException;

import static org.lwjgl.opengl.GL11.glViewport;

public class GameEngine implements Runnable {
    public static final int TARGET_FPS = 75;
    public static final int TARGET_UPS = 30;

    private Window window;
    private Thread loopThread;
    private boolean verbose;

    private DocumentHandler documentHandler;
    private Document document;

    public GameEngine(String title, int width, int height, DocumentHandler initHandler, boolean verbose) throws IOException {
        this.verbose = verbose;

        if (this.verbose) Logger.log("Creating window...");
        window = new Window(title, width, height);
        this.documentHandler = initHandler;
    }

    public void start() throws IOException {
        if (this.verbose) Logger.log("Starting loop...");
        if (System.getProperty("os.name").contains("Mac")) {
            this.run();
        } else {
            loopThread = new Thread(this, "LOOP_THREAD");
            loopThread.start();
        }
    }

    @Override
    public void run() {
        try {
            init();
            gameLoop();
        } catch (Exception exc) {
            exc.printStackTrace();
        } finally {
            cleanup();
        }
    }

    private void gameLoop() throws Exception {
        long last = 0;

        if (this.verbose) Logger.log("Doing first render...");
        update(0);
        render();
        window.show();
        if (this.verbose) Logger.log("Done first render.");

        boolean running = true;
        while (running && !window.windowShouldClose()) {
            long startTime = Timer.getTime();

            update((int) (startTime - last));
            last = startTime;
            render();
            sync(startTime);

            Timer.update();
        }
        if (this.verbose) {
            Logger.log("Stopping on tick index: " + Timer.getGameLoopIndex());
        }
    }

    private void render() throws Exception {
        document.render(window);
        window.update();
    }

    private void update(int interval) throws Exception {
        DocumentHandler newDoc = documentHandler.update(window, document, interval);
        documentHandler.mouseInput.input();
        if (newDoc != null) {
            documentHandler = newDoc;
            this.loadDocument();
        }
    }

    private void loadDocument() throws Exception {
        documentHandler.mouseInput.init(window);
        documentHandler.keyInput.init(window);
        this.document = documentHandler.init(window);
        this.window.setClearColor(this.document.background);
    }

    private void sync(long startTime) throws InterruptedException {
        int length = (int) (Timer.getTime() - startTime);
        int target = 1000 / TARGET_FPS;
        int loops = target - length;
        for (int i = 0; i < loops && i >= 0; i++) {
            Thread.sleep(1);
        }
    }

    private void init() throws Exception {
        if (this.verbose) Logger.log("Initialising...");
        window.init();
        this.loadDocument();
    }

    private void cleanup() {
        document.cleanup();
        documentHandler.cleanup();
    }
}
package net.comsoria.engine;

import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.input.KeyInput;
import net.comsoria.engine.view.input.MouseInput;

public interface IGameLogic {
    void init(Window window, KeyInput keyInput) throws Exception;

    void update(Window window, float interval, MouseInput mouse, KeyInput keys);
    void render(Window window) throws Exception;

    void cleanup();
}
package net.comsoria.engine.libraryImplementation;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;

public interface AudioLibrary {
    AudioLibrary AL = new OpenAL();

    void bufferData(int name, Format format, ShortBuffer data, int frequency);
    void deleteBuffers(int buffer);
    int genBuffer();
    long openDevice(ByteBuffer data);
    int createCapabilites(long device);
    long createContext(long device, IntBuffer data);
    void makeContextCurrent(long context);
    void createCapabilites(int deviceCaps);
    void destroyContext(long context);
    void closeDevice(long device);
    void setDistanceModel(Model model);
    void listener(Parameter param, float x, float y, float z);
    void listener(Parameter param, float[] values);
    int genSource();
    void setSourceProperty(int id, SourceParam param, boolean value);
    void setSourceProperty(int id, SourceParam param, int value);
    void setSourceProperty(int id, SourceParam param, float value);
    void setSourceProperty(int id, SourceParam param, float x, float y, float z);
    void play(int source);
    void pause(int source);
    void stop(int source);
    void delete(int source);
    boolean isPlaying(int source);

    enum Format {
        MONO_16,
        STEREO_16
    }

    enum Parameter {
        Position,
        Velocity,
        Orientation
    }

    enum SourceParam {
        Looping,
        Relative,
        Buffer,
        Position,
        Velocity,
        Gain
    }

    enum Model {
        Exponent
    }
}
package net.comsoria.engine.libraryImplementation;

import org.lwjgl.openal.ALC;
import org.lwjgl.openal.ALCCapabilities;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.openal.AL10.*;
import static org.lwjgl.openal.AL11.AL_EXPONENT_DISTANCE;
import static org.lwjgl.openal.ALC10.*;

public class OpenAL implements AudioLibrary {
    private List<ALCCapabilities> capabilities = new ArrayList<>();

    @Override public void bufferData(int name, Format format, ShortBuffer data, int frequency) {
        alBufferData(
                name,
                format == Format.MONO_16? AL_FORMAT_MONO16:AL_FORMAT_STEREO16,
                data,
                frequency
        );
    }

    @Override public void deleteBuffers(int buffer) {
        alDeleteBuffers(buffer);
    }
    @Override public int genBuffer() {
        return alGenBuffers();
    }

    @Override public long openDevice(ByteBuffer data) {
        return alcOpenDevice(data);
    }
    @Override public int createCapabilites(long device) {
        this.capabilities.add(ALC.createCapabilities(device));
        return this.capabilities.size() - 1;
    }
    @Override public long createContext(long device, IntBuffer data) {
        return alcCreateContext(device, data);
    }
    @Override public void makeContextCurrent(long context) {
        alcMakeContextCurrent(context);
    }
    @Override public void createCapabilites(int deviceCaps) {
        org.lwjgl.openal.AL.createCapabilities(this.capabilities.get(deviceCaps));
    }

    @Override public void destroyContext(long context) {
        alcDestroyContext(context);
    }
    @Override public void closeDevice(long device) {
        alcCloseDevice(device);
    }

    private int getModelInt(Model model) {
        switch (model) {
            case Exponent:
                return AL_EXPONENT_DISTANCE;
        }

        return -1;
    }
    @Override public void setDistanceModel(Model model) {
        alDistanceModel(getModelInt(model));
    }

    private int getParamInt(Parameter parameter) {
        switch (parameter) {
            case Position:
                return AL_POSITION;
            case Velocity:
                return AL_VELOCITY;
            case Orientation:
                return AL_ORIENTATION;
        }

        return -1;
    }
    @Override public void listener(Parameter param, float x, float y, float z) {
        alListener3f(getParamInt(param), x, y, z);
    }
    @Override public void listener(Parameter param, float[] values) {
        alListenerfv(getParamInt(param), values);
    }

    @Override public int genSource() {
        return alGenSources();
    }

    private int getParamInt(SourceParam parameter) {
        switch (parameter) {
            case Looping:
                return AL_LOOPING;
            case Relative:
                return AL_SOURCE_RELATIVE;
            case Buffer:
                return AL_BUFFER;
            case Position:
                return AL_POSITION;
            case Velocity:
                return AL_VELOCITY;
            case Gain:
                return AL_GAIN;
        }

        return -1;
    }
    @Override public void setSourceProperty(int id, SourceParam param, boolean value) {
        this.setSourceProperty(id, param, value? AL_TRUE:AL_FALSE);
    }
    @Override public void setSourceProperty(int id, SourceParam param, int value) {
        alSourcei(id, getParamInt(param), value);
    }
    @Override public void setSourceProperty(int id, SourceParam param, float value) {
        alSourcef(id, getParamInt(param), value);
    }
    @Override public void setSourceProperty(int id, SourceParam param, float x, float y, float z) {
        alSource3f(id, getParamInt(param), x, y, z);
    }

    @Override public void play(int source) {
        alSourcePlay(source);
    }
    @Override public void pause(int source) {
        alSourcePause(source);
    }
    @Override public void stop(int source) {
        alSourceStop(source);
    }

    @Override public void delete(int source) {
        alDeleteSources(source);
    }

    @Override
    public boolean isPlaying(int source) {
        return alGetSourcei(source, AL_SOURCE_STATE) == AL_PLAYING;
    }
}
package net.comsoria.engine.loaders;

import java.util.ArrayList;
import java.util.List;

public class CSVLoader {
    private Row[] rows;

    public CSVLoader(String text) {
        this.load(text.split("\n"));
    }

    public CSVLoader(String[] lines) {
        this.load(lines);
    }

    public void load(String[] lines) {
        List<Row> rows = new ArrayList<>();
        for (String line : lines) {
            if (!line.trim().equals("")) {
                rows.add(new Row(line.replace("\"", "").split(",")));
            }
        }

        this.rows = new Row[rows.size()];
        for (int i = 0; i < rows.size(); i++)
            this.rows[i] = rows.get(i);
    }

    public String get(int x, int y) {
        return rows[y].getPart(x);
    }

    public Row getRow(int y) {
        return rows[y];
    }

    public int rows() {
        return rows.length;
    }

    public static class Row {
        private final String[] parts;

        public Row(String[] parts) {
            this.parts = parts;
        }

        public String getPart(int id) {
            return parts[id].trim();
        }

        @Override
        public String toString() {
            String text = "";
            for (String string : this.parts) {
                text += "\"" + string + "\",";
            }
            return text.replaceAll(",$", "");
        }
    }

    @Override public String toString() {
        String text = "";
        for (Row row : this.rows) {
            text += row.toString() + "\n";
        }
        return text.trim();
    }
}
package net.comsoria.engine.loaders;

import net.comsoria.engine.utils.Utils;
import org.lwjgl.BufferUtils;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.channels.Channels;
import java.nio.channels.ReadableByteChannel;
import java.nio.channels.SeekableByteChannel;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.StringJoiner;

import static org.lwjgl.BufferUtils.createByteBuffer;

public class FileLoader {
    public static InputStream loadResourceAsStream(String path) throws FileNotFoundException {
        return new FileInputStream(new File(path));
    }

    public static InputStream loadResourceAsStream(File file) throws FileNotFoundException {
        return new FileInputStream(file);
    }

    public static List<String> loadResourceLines(String path) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(loadResourceAsStream(path)));
        List<String> lines = new ArrayList<>();

        String line;
        while ((line = reader.readLine()) != null) {
            lines.add(line);
        }

        return lines;
    }

    public static String loadResourceAsStringFromStream(InputStream stream) throws IOException {
        StringJoiner stringBuilder = new StringJoiner(System.getProperty("line.separator"));
        BufferedReader reader = new BufferedReader(new InputStreamReader(stream));

        String inputLine;
        while ((inputLine = reader.readLine()) != null)
            stringBuilder.add(inputLine);
        reader.close();
        stream.close();

        return stringBuilder.toString();
    }

    public static String loadResourceAsStringFromPath(String path) throws IOException {
        return loadResourceAsString(Utils.utils.getPath(path));
    }

    public static String loadResourceAsString(String path) throws IOException {
        return loadResourceAsStringFromStream(loadResourceAsStream(path));
    }

    public static String loadResourceAsString(File file) throws IOException {
        return loadResourceAsStringFromStream(loadResourceAsStream(file));
    }

    public static void writeResource(String path, String text) throws IOException {
        writeResource(new File(path), text);
    }

    public static void writeResource(File file, String text) throws IOException {
        FileWriter writer = new FileWriter(file);
        writer.write(text);
        writer.close();
    }

    public boolean fileExists(String path) {
        return new File(path).exists();
    }

    public static ByteBuffer ioResourceToByteBuffer(String resource, int bufferSize) throws IOException {
        ByteBuffer buffer;

        Path path = Paths.get(resource);
        if (Files.isReadable(path)) {
            try (SeekableByteChannel fc = Files.newByteChannel(path)) {
                buffer = createByteBuffer((int) fc.size() + 1);
                while (fc.read(buffer) != -1) ;
            }
        } else {
            try (
                    InputStream source = Utils.class.getResourceAsStream(resource);
                    ReadableByteChannel rbc = Channels.newChannel(source)) {
                buffer = createByteBuffer(bufferSize);

                while (true) {
                    int bytes = rbc.read(buffer);
                    if (bytes == -1) {
                        break;
                    }
                    if (buffer.remaining() == 0) {
                        buffer = Utils.resizeBuffer(buffer, buffer.capacity() * 2);
                    }
                }
            }
        }

        buffer.flip();
        return buffer;
    }
}
package net.comsoria.engine.loaders;

import net.comsoria.engine.utils.Logger;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class GLSLLoader {
    public static String loadGLSL(String path, Map<String,String> constants) throws IOException {
        String text = FileLoader.loadResourceAsString(path);
        File file = new File(path);

        final String noLines = text.replace("\n", "");

        for (String key : constants.keySet()) {
            String value = constants.get(key);

            String pattern = "uniform ([a-zA-Z0-9]+) " + key + ";";
            Matcher m = Pattern.compile(pattern).matcher(noLines);
            if (m.find()) {
                text = text.replaceAll(pattern, "const " + m.group(1) + " " + key + " = " + value + ";");
            } else {
                pattern = "const ([a-zA-Z0-9]+) " + key + ";";
                m = Pattern.compile(pattern).matcher(noLines);
                if (m.find()) {
                    text = text.replaceAll(pattern, "const " + m.group(1) + " " + key + " = " + value + ";");
                } else {
                    Logger.log("Constant of name '" + key + "' not found in '" + file.getName() + "'", Logger.LogType.WARN);
                }
            }
        }

        return text;
    }
}
package net.comsoria.engine.loaders;

public class LoaderException extends RuntimeException {
    public LoaderException(String msg) {
        super(msg);
    }
}
package net.comsoria.engine.loaders;

import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.view.graph.BufferAttribute;
import org.joml.Vector2f;
import org.joml.Vector3f;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class OBJLoader {
    public static Tuple<List<BufferAttribute>, int[]> loadGeometry(String fileName) throws IOException {
        List<String> lines = FileLoader.loadResourceLines(fileName);

        List<Vector3f> vertices = new ArrayList<>();
        List<Vector2f> textures = new ArrayList<>();
        List<Vector3f> normals = new ArrayList<>();
        List<Face> faces = new ArrayList<>();

        for (String line : lines) {
            String[] tokens = line.split("\\s+");

            switch (tokens[0]) {
                case "v":
                    // Geometric vertex
                    Vector3f vec3f = new Vector3f(
                            Float.parseFloat(tokens[1]),
                            Float.parseFloat(tokens[2]),
                            Float.parseFloat(tokens[3]));
                    vertices.add(vec3f);
                    break;
                case "vt":
                    // Texture coordinate
                    Vector2f vec2f = new Vector2f(
                            Float.parseFloat(tokens[1]),
                            Float.parseFloat(tokens[2]));
                    textures.add(vec2f);
                    break;
                case "vn":
                    // Vertex normal
                    Vector3f vec3fNorm = new Vector3f(
                            Float.parseFloat(tokens[1]),
                            Float.parseFloat(tokens[2]),
                            Float.parseFloat(tokens[3]));
                    normals.add(vec3fNorm);
                    break;
                case "f": // Face
                    Face face = new Face(tokens[1], tokens[2], tokens[3]);
                    faces.add(face);
                    break;
                default:
                    // Ignore other lines
                    break;
            }
        }
        return reorderLists(vertices, textures, normals, faces);
    }

    private static Tuple<List<BufferAttribute>, int[]> reorderLists(List<Vector3f> posList, List<Vector2f> textCoordList,
                                                                     List<Vector3f> normList, List<Face> facesList) {

        List<Integer> indices = new ArrayList();
        // Create position array in the order it has been declared
        float[] posArr = new float[posList.size() * 3];
        int i = 0;
        for (Vector3f pos : posList) {
            posArr[i * 3] = pos.x;
            posArr[i * 3 + 1] = pos.y;
            posArr[i * 3 + 2] = pos.z;
            i++;
        }
        float[] textCoordArr = new float[posList.size() * 2];
        float[] normArr = new float[posList.size() * 3];

        for (Face face : facesList) {
            IdxGroup[] faceVertexIndices = face.getFaceVertexIndices();
            for (IdxGroup indValue : faceVertexIndices) {
                processFaceVertex(indValue, textCoordList, normList,
                        indices, textCoordArr, normArr);
            }
        }
        int[] indicesArr = indices.stream().mapToInt((Integer v) -> v).toArray();
        List<BufferAttribute> attributes = new ArrayList<>();
        attributes.add(new BufferAttribute(posArr, 3));
        attributes.add(new BufferAttribute(textCoordArr, 2));
        attributes.add(new BufferAttribute(normArr, 3));

        return new Tuple<>(attributes, indicesArr);
    }

    private static void processFaceVertex(IdxGroup indices, List<Vector2f> textCoordList,
                                          List<Vector3f> normList, List<Integer> indicesList,
                                          float[] texCoordArr, float[] normArr) {

        // Set index for vertex coordinates
        int posIndex = indices.idxPos;
        indicesList.add(posIndex);

        // Reorder texture coordinates
        if (indices.idxTextCoord >= 0) {
            Vector2f textCoord = textCoordList.get(indices.idxTextCoord);
            texCoordArr[posIndex * 2] = textCoord.x;
            texCoordArr[posIndex * 2 + 1] = 1 - textCoord.y;
        }
        if (indices.idxVecNormal >= 0) {
            // Reorder vectornormals
            Vector3f vecNorm = normList.get(indices.idxVecNormal);
            normArr[posIndex * 3] = vecNorm.x;
            normArr[posIndex * 3 + 1] = vecNorm.y;
            normArr[posIndex * 3 + 2] = vecNorm.z;
        }
    }

    private static class Face {
        private IdxGroup[] idxGroups = new IdxGroup[3];

        public Face(String v1, String v2, String v3) {
            idxGroups = new IdxGroup[3];
            // Parse the lines
            idxGroups[0] = parseLine(v1);
            idxGroups[1] = parseLine(v2);
            idxGroups[2] = parseLine(v3);
        }

        private IdxGroup parseLine(String line) {
            IdxGroup idxGroup = new IdxGroup();

            String[] lineTokens = line.split("/");
            int length = lineTokens.length;
            idxGroup.idxPos = Integer.parseInt(lineTokens[0]) - 1;
            if (length > 1) {
                // It can be empty if the obj does not define text coords
                String textCoord = lineTokens[1];
                idxGroup.idxTextCoord = textCoord.length() > 0 ? Integer.parseInt(textCoord) - 1 : IdxGroup.NO_VALUE;
                if (length > 2) {
                    idxGroup.idxVecNormal = Integer.parseInt(lineTokens[2]) - 1;
                }
            }

            return idxGroup;
        }

        public IdxGroup[] getFaceVertexIndices() {
            return idxGroups;
        }
    }

    protected static class IdxGroup {
        public static final int NO_VALUE = -1;
        public int idxPos;
        public int idxTextCoord;
        public int idxVecNormal;

        public IdxGroup() {
            idxPos = NO_VALUE;
            idxTextCoord = NO_VALUE;
            idxVecNormal = NO_VALUE;
        }
    }
}
package net.comsoria.engine.loaders;

import net.comsoria.engine.utils.random.Random;
import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.graph.BufferAttribute;
import org.joml.*;

import java.lang.Math;
import java.util.*;

public final class Shape {
    public static Tuple<List<BufferAttribute>, int[]> genRect(float w, float h) {
        w *= 0.5;
        h *= 0.5;

        BufferAttribute positions = new BufferAttribute(new float[]{
                -w, -h, 0,
                -w, h, 0,
                w, h, 0,
                w, -h, 0
        }, 3);

        BufferAttribute texCoords = new BufferAttribute(new float[]{
                0, 0,
                0, 1,
                1, 1,
                1, 0,
        }, 2);

       return new Tuple<>(Arrays.asList(positions, texCoords), new int[] {
               0, 1, 2, 0, 2, 3
       });
    }

    private static List<Vector3f> genCircle(int c, float r, Vector3f offset) {
        float angle = (float) ((2 * Math.PI) / c);

        List<Vector3f> result = new ArrayList<>();
        for (int i = 0; i < c + 1; i++) {
            result.add(new Vector3f(offset).add(
                    (float) Math.sin(i * angle) * r,
                    0,
                    (float) Math.cos(i * angle) * r
            ));
        }

        return result;
    }

    public static Tuple<List<BufferAttribute>, int[]> genCylinder(int c, int h, float r) {
        return genCylinder(c, h, new CylinderGenerator() {
            @Override
            public float radius(int h) {
                return r;
            }

            @Override
            public Vector3f offset(int h) {
                return new Vector3f(0, h, 0);
            }
        });
    }

    public static Tuple<List<BufferAttribute>, int[]> genCylinder(int c, int h, CylinderGenerator generator) {
        List<Vector3f> points = new ArrayList<>();
        List<Vector3i> faces = new ArrayList<>();
        List<Vector2f> planePoints = new ArrayList<>();

        points.addAll(genCircle(c, generator.radius(0), generator.offset(0)));
        final int pointsPerCircle = points.size();

        for (int i = 0; i < pointsPerCircle; i++) planePoints.add(new Vector2f(i, 0));

        for (int i = 1; i < h; i++) {
            points.addAll(genCircle(c, generator.radius(i), generator.offset(i)));

            for (int x = 0; x < pointsPerCircle; x++) planePoints.add(new Vector2f(x, i));

            for (int x = 0; x < c; x++) {
                faces.add(new Vector3i(
                        ((i - 1) * pointsPerCircle) + x,
                        (i * pointsPerCircle) + x,
                        (i * pointsPerCircle) + x + 1
                ));

                faces.add(new Vector3i(
                        ((i - 1) * pointsPerCircle) + x,
                        (i * pointsPerCircle) + x + 1,
                        ((i - 1) * pointsPerCircle) + x + 1
                ));
            }
        }

        int[] indices = new int[faces.size() * 3];
        for (int i = 0; i < faces.size(); i++) {
            Vector3i face = faces.get(i);
            indices[(i * 3)] = face.x;
            indices[(i * 3) + 1] = face.y;
            indices[(i * 3) + 2] = face.z;
        }

        return new Tuple<>(new ArrayList<>(Arrays.asList(
                BufferAttribute.create3f(points), BufferAttribute.create2f(planePoints)
        )), indices);
    }

    public static Tuple<List<BufferAttribute>, int[]> genSphere(int p, float radius) {
        return genCylinder(p, p + 1, new CylinderGenerator() {
            @Override
            public float radius(int h) {
                return (float) Math.sin((Math.PI / p) * (float) h) * radius;
            }

            @Override
            public Vector3f offset(int h) {
                return new Vector3f(0, ((float) Math.cos((Math.PI / p) * (float) h) * radius), 0);
            }
        });
    }

    public static BufferAttribute generateDisplacement(BufferAttribute points, float scale, int d) {
        return generateDisplacement(points, (x, y, z) -> scale, d);
    }

    public static BufferAttribute generateSphereDisplacement(BufferAttribute points, float scale, int d, float radius) {
        return generateDisplacement(points, (x, y, z) -> (Utils.distance(x, z) / radius) * scale, d);
    }

    public static BufferAttribute generateDisplacement(BufferAttribute points, DisplacementGenerator generator, int d) {
        int parts = points.parts();

        float[] data = new float[parts * d];

        for (int i = 0; i < parts; i++) {
            float x = Utils.round(points.get(i * 3), 2);
            float y = Utils.round(points.get((i * 3) + 1), 2);
            float z = Utils.round(points.get((i * 3) + 2), 2);

            for (int a = 0; a < d; a++) {
                data[(i * d) + a] = Random.random.noise(x, y, z, 23.65f * a) * generator.scale(x, y, z);
            }
        }

        return new BufferAttribute(data, d);
    }

    public static void reanchor(BufferAttribute points, float x, float y, float z) {
        float[] min = new float[] {0, 0, 0};
        float[] max = new float[] {0, 0, 0};

        for (int i = 0; i < points.parts(); i++) {
            Vector3f vec = points.getVec3(i);

            if (vec.x < min[0]) min[0] = vec.x;
            if (vec.y < min[1]) min[1] = vec.y;
            if (vec.z < min[2]) min[2] = vec.z;

            if (vec.x > max[0]) max[0] = vec.x;
            if (vec.y > max[1]) max[1] = vec.y;
            if (vec.z > max[2]) max[2] = vec.z;
        }

        float[] diff = new float[] {max[0] - min[0], max[1] - min[1], max[2] - min[2]};

        points.translate(
                -((diff[0] * x) + min[0]),
                -((diff[1] * y) + min[1]),
                -((diff[2] * z) + min[2])
        );
    }

    public interface DisplacementGenerator {
        float scale(float x, float y, float z);
    }

    public interface CylinderGenerator {
        float radius(int h);
        Vector3f offset(int h);
    }
}
package net.comsoria.engine.loaders.text;

import net.comsoria.engine.view.graph.Texture;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.InputStream;
import java.nio.charset.Charset;
import java.nio.charset.CharsetEncoder;
import java.util.HashMap;
import java.util.Map;

public class FontTexture {
    private static final String IMAGE_FORMAT = "png";
    private final Font font;
    private final String charSetName;
    private final Map<Character, CharInfo> charMap;
    private Texture texture;

    private int height;

    private int width;

    public FontTexture(Font font, String charSetName) throws Exception {
        this.font = font;
        this.charSetName = charSetName;
        charMap = new HashMap<>();

        buildTexture();
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public Texture getTexture() {
        return texture;
    }

    public CharInfo getCharInfo(char c) {
        return charMap.get(c);
    }

    private String getAllAvailableChars(String charsetName) {
        CharsetEncoder ce = Charset.forName(charsetName).newEncoder();
        StringBuilder result = new StringBuilder();
        for (char c = 0; c < Character.MAX_VALUE; c++) {
            if (ce.canEncode(c)) {
                result.append(c);
            }
        }
        return result.toString();
    }

    private void buildTexture() throws Exception {
        // Get the font metrics for each character for the selected font by using image
        BufferedImage img = new BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2D = img.createGraphics();
        g2D.setFont(font);
        FontMetrics fontMetrics = g2D.getFontMetrics();

        String allChars = getAllAvailableChars(charSetName);
        this.width = 0;
        this.height = 0;
        for (char c : allChars.toCharArray()) {
            // Get the size for each character and update global image size
            CharInfo charInfo = new CharInfo(width, fontMetrics.charWidth(c));
            charMap.put(c, charInfo);
            width += charInfo.getWidth();
            height = Math.max(height, fontMetrics.getHeight());
        }
        g2D.dispose();

        // Create the image associated to the charset
        img = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        g2D = img.createGraphics();
        g2D.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        g2D.setFont(font);
        fontMetrics = g2D.getFontMetrics();
        g2D.setColor(Color.WHITE);
        g2D.drawString(allChars, 0, fontMetrics.getAscent());
        g2D.dispose();

        // Dump image to a byte buffer
        InputStream is;
        try (ByteArrayOutputStream out = new ByteArrayOutputStream()) {
            ImageIO.write(img, IMAGE_FORMAT, out);
            out.flush();
            is = new ByteArrayInputStream(out.toByteArray());
        }

        texture = new Texture(is);
    }

    public static class CharInfo {

        private final int startX;

        private final int width;

        public CharInfo(int startX, int width) {
            this.startX = startX;
            this.width = width;
        }

        public int getStartX() {
            return startX;
        }

        public int getWidth() {
            return width;
        }
    }
}
package net.comsoria.engine.loaders.text;

import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.GLSL.programs.ShaderProgram2D;
import net.comsoria.engine.view.graph.BufferAttribute;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.mesh.Mesh;
import net.comsoria.engine.view.graph.mesh.Mesh2D;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TextLoader {
    private static final float ZPOS = 0.0f;
    private static final int VERTICES_PER_QUAD = 4;

    public static Tuple<List<BufferAttribute>, int[]> buildGeometryData(String text, FontTexture texture) {
        List<Float> positions = new ArrayList<>();
        List<Float> textCoords = new ArrayList<>();
        List<Integer> indices   = new ArrayList<>();
        char[] characters = text.toCharArray();
        int numChars = characters.length;

        float startx = 0;
        for(int i = 0; i < numChars; i++) {
            FontTexture.CharInfo charInfo = texture.getCharInfo(characters[i]);

            // Build a character tile composed by two triangles

            // Left Top vertex
            positions.add(startx); // x
            positions.add(0.0f); //y
            positions.add(ZPOS); //z
            textCoords.add( (float)charInfo.getStartX() / (float)texture.getWidth());
            textCoords.add(0.0f);
            indices.add(i * VERTICES_PER_QUAD);

            // Left Bottom vertex
            positions.add(startx); // x
            positions.add((float)texture.getHeight()); //y
            positions.add(ZPOS); //z
            textCoords.add((float)charInfo.getStartX() / (float)texture.getWidth());
            textCoords.add(1.0f);
            indices.add((i * VERTICES_PER_QUAD) + 1);

            // Right Bottom vertex
            positions.add(startx + charInfo.getWidth()); // x
            positions.add((float)texture.getHeight()); //y
            positions.add(ZPOS); //z
            textCoords.add((float) (charInfo.getStartX() + charInfo.getWidth()) / (float) texture.getWidth());
            textCoords.add(1.0f);
            indices.add((i * VERTICES_PER_QUAD) + 2);

            // Right Top vertex
            positions.add(startx + charInfo.getWidth()); // x
            positions.add(0.0f); //y
            positions.add(ZPOS); //z
            textCoords.add((float) (charInfo.getStartX() + charInfo.getWidth()) / (float)texture.getWidth());
            textCoords.add(0.0f);
            indices.add((i * VERTICES_PER_QUAD) + 3);

            // Add indices por left top and bottom right vertices
            indices.add(i * VERTICES_PER_QUAD);
            indices.add((i * VERTICES_PER_QUAD) + 2);

            startx += charInfo.getWidth();
        }

        int[] indicesArr = indices.stream().mapToInt(i -> i).toArray();

        BufferAttribute pos = new BufferAttribute(Utils.listToArray(positions), 3);
        BufferAttribute texCoords = new BufferAttribute(Utils.listToArray(textCoords), 2);

        return new Tuple<>(new ArrayList<>(Arrays.asList(pos, texCoords)), indicesArr);
    }

    public static Mesh buildMesh(String text, FontTexture texture) throws IOException {
        Geometry geometry = new Geometry(buildGeometryData(text, texture));

        Mesh mesh = new Mesh2D(geometry, new Material(), new ShaderProgram2D());
        mesh.initShaderProgram();
        mesh.material.textures.add(texture.getTexture());
        mesh.shaderProgram.createTextureUniform("texture_sampler");
        mesh.material.ambientColour.set(1f, 1f, 1f, 1f);
        mesh.position.z -= 100;

        return mesh;
    }
}
package net.comsoria.engine.loaders;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;

public class WebLoader {
    public static String loadResourceFromNet(String resourceName) throws IOException {
        URL resource = new URL(resourceName);
        return FileLoader.loadResourceAsStringFromStream(resource.openStream());
    }

    public static void copyImageFromNet(String imageUrl, String destinationFile) throws IOException {
        URL url = new URL(imageUrl);
        InputStream is = url.openStream();
        OutputStream os = new FileOutputStream(destinationFile);

        byte[] b = new byte[2048];
        int length;

        while ((length = is.read(b)) != -1) {
            os.write(b, 0, length);
        }

        is.close();
        os.close();
    }

    public static boolean netIsAvailable() {
        try {
            URL url = new URL("http://www.google.com");
            url.openStream();
            return true;
        } catch (Exception e) {
            return false;
        }
    }
}
package net.comsoria.engine.loaders.xhtml.ui;

import net.comsoria.engine.view.FrameBufferRenderer;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.Renderer;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.color.Color3;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Document {
    public final List<UINode> nodeList = new ArrayList<>();
    public Color3 background = Color3.WHITE.clone();

    public final FrameBufferRenderer frameBufferRenderer = new FrameBufferRenderer();

    private Transformation transformation = new Transformation();

    public Document() throws IOException {

    }

    public UINode getElementByID(String id) {
        for (UINode node : nodeList) {
            String elID = node.originalXML.getParam("id");
            if (elID != null && elID.equals(id)) return node;
        }

        return null;
    }

    public void addNode(UINode node, Window window) throws Exception {
        this.nodeList.add(node);

        node.genMesh(window);
    }

    public void updateAllStyleSets(Window window) throws Exception {
        for (UINode node : this.nodeList) {
            node.updateStyleSets(window);
        }
    }

    public void render(Window window) throws Exception {
        frameBufferRenderer.renderBase(window);
        frameBufferRenderer.bindInitialFramebuffer();

        transformation.getOrthoProjectionMatrix(0, window.getWidth(), window.getHeight(), 0);

        Renderer.render(nodeList, null, transformation, window);

        frameBufferRenderer.renderFramebuffers(window);
    }

    public void cleanup() {
        for (UINode node : this.nodeList)
            if (node.getMesh() != null) node.getMesh().cleanup();
    }
}
package net.comsoria.engine.loaders.xhtml.ui;

import net.comsoria.engine.view.FrameBufferRenderer;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.input.KeyInput;
import net.comsoria.engine.view.input.MouseInput;

import java.io.IOException;

public abstract class DocumentHandler {
    public final KeyInput keyInput = new KeyInput();
    public final MouseInput mouseInput = new MouseInput();

    public abstract Document init(Window window) throws Exception;
    public abstract void cleanup();
    public abstract DocumentHandler update(Window window, Document document, float interval) throws Exception;
}
package net.comsoria.engine.loaders.xhtml.ui.node;

import net.comsoria.engine.Scene;
import net.comsoria.engine.loaders.xhtml.ui.StyleSet;
import net.comsoria.engine.loaders.xhtml.ui.UINode;
import net.comsoria.engine.loaders.xml.XMLNode;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.Renderable;
import net.comsoria.engine.view.Renderer;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.engine.view.graph.mesh.Mesh2D;

import java.io.Closeable;

public class Canvas extends UINode {
    public Scene scene = new Scene();

    public Canvas(StyleSet styleSet, XMLNode xmlNode) {
        super(styleSet, xmlNode);
    }

    @Override
    protected void genMesh(Window window) throws Exception {
        this.mesh = null;
    }

    @Override
    public Closeable render(Transformation transformation, Scene s, RenderData renderData, Window window) throws Exception {
        transformation.getView(scene.camera);
        transformation.getProjectionMatrix(window, scene.camera);
        transformation.getViewNoRotation(scene.camera);
        transformation.getViewNoTranslation(scene.camera);

        Renderer.setViewPort(
                0, //TODO: turn into real values
                0,
                (int) (Float.valueOf(styleSet.ruleMap.get("width").value) * (window.getWidth() / 100f)),
                (int) (Float.valueOf(styleSet.ruleMap.get("height").value) * (window.getHeight() / 100f))
        );

        scene.render(transformation, window);

        return null;
    }

    @Override
    public RenderOrder getRenderOrder() {
        return RenderOrder.First;
    }

    @Override
    public void cleanup() {
        super.cleanup();

        scene.cleanup();
    }
}
package net.comsoria.engine.loaders.xhtml.ui.node;

import net.comsoria.engine.loaders.OBJLoader;
import net.comsoria.engine.loaders.xhtml.ui.StyleSet;
import net.comsoria.engine.loaders.xhtml.ui.UINode;
import net.comsoria.engine.loaders.xml.XMLNode;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.GLSL.programs.ShaderProgram2D;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.mesh.Mesh2D;

public class Model extends UINode {
    public Model(StyleSet styleSet, XMLNode xmlNode) {
        super(styleSet, xmlNode);
    }

    @Override
    protected void genMesh(Window window) throws Exception {
        mesh = new Mesh2D(new Geometry(OBJLoader.loadGeometry(Utils.utils.p(this.originalXML.getParam("path")))), new Material(), new ShaderProgram2D());
        mesh.initShaderProgram();
    }

    @Override
    public RenderOrder getRenderOrder() {
        return RenderOrder.End;
    }
}
package net.comsoria.engine.loaders.xhtml.ui.node;

import net.comsoria.engine.loaders.Shape;
import net.comsoria.engine.loaders.text.FontTexture;
import net.comsoria.engine.loaders.text.TextLoader;
import net.comsoria.engine.loaders.xhtml.ui.UINode;
import net.comsoria.engine.loaders.xhtml.ui.StyleSet;
import net.comsoria.engine.loaders.xml.XMLNode;
import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.GLSL.programs.ShaderProgram2D;
import net.comsoria.engine.view.Renderable;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.graph.BufferAttribute;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.mesh.Mesh;
import net.comsoria.engine.view.graph.mesh.Mesh2D;

import java.awt.*;
import java.io.File;
import java.util.List;

public class Paragraph extends UINode {
    public Paragraph(StyleSet styleSet, XMLNode xmlNode) {
        super(styleSet, xmlNode);
    }

    @Override
    protected void genMesh(Window window) throws Exception {
        Font font = Font.createFont(Font.TRUETYPE_FONT, new File(Utils.utils.p(styleSet.ruleMap.get("font").value))).deriveFont(90f);
        FontTexture texture = new FontTexture(font, "ISO-8859-1");

        Tuple<List<BufferAttribute>, int[]> geomData = TextLoader.buildGeometryData(originalXML.getInnerText(), texture);

        String offsetString = originalXML.getParam("anchor");
        if (offsetString != null) {
            String[] parts = offsetString.split(" ");

            Shape.reanchor(geomData.getA().get(0), Float.valueOf(parts[0]), Float.valueOf(parts[1]), Float.valueOf(parts[2]));
        }
        Geometry geometry = new Geometry(geomData);

        this.mesh = new Mesh2D(geometry, new Material(), new ShaderProgram2D());
        mesh.initShaderProgram();
        mesh.material.textures.add(texture.getTexture());
        mesh.shaderProgram.createTextureUniform("texture_sampler");
    }

    @Override
    public RenderOrder getRenderOrder() {
        return RenderOrder.End;
    }
}
package net.comsoria.engine.loaders.xhtml.ui.node;

import net.comsoria.engine.loaders.Shape;
import net.comsoria.engine.loaders.xhtml.ui.StyleSet;
import net.comsoria.engine.loaders.xhtml.ui.UINode;
import net.comsoria.engine.loaders.xml.XMLNode;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.GLSL.programs.ShaderProgram2D;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.Texture;
import net.comsoria.engine.view.graph.mesh.Mesh2D;

public class Rectangle extends UINode {
    public Rectangle(StyleSet styleSet, XMLNode xmlNode) {
        super(styleSet, xmlNode);
    }

    @Override
    protected void genMesh(Window window) throws Exception {
        Geometry geometry = new Geometry(Shape.genRect(
                Float.valueOf(styleSet.ruleMap.get("width").value) * (window.getWidth() / 100f),
                Float.valueOf(styleSet.ruleMap.get("height").value) * (window.getHeight() / 100f)
        ));
        mesh = new Mesh2D(geometry, new Material(), new ShaderProgram2D());
        mesh.initShaderProgram();

        String texturePath = originalXML.getParam("imageSrc");
        if (texturePath != null) {
            Texture texture = new Texture(Utils.utils.p(texturePath));
            mesh.material.textures.add(texture);
            mesh.shaderProgram.createTextureUniform("texture_sampler");
        }
    }

    @Override
    public RenderOrder getRenderOrder() {
        return RenderOrder.First;
    }
}
package net.comsoria.engine.loaders.xhtml.ui;

import net.comsoria.engine.utils.Logger;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.color.Color3;
import net.comsoria.engine.view.color.Color4;
import net.comsoria.engine.view.graph.Texture;
import net.comsoria.engine.view.graph.mesh.Mesh;
import net.comsoria.engine.view.graph.mesh.Mesh2D;
import org.joml.Vector2f;

import javax.swing.text.Style;
import java.util.*;

public class StyleSet {
    public final static StyleSet defaultStyleSet = new StyleSet();

    public Map<String, StyleRule> ruleMap = new HashMap<>();

    private void initRuleMap() {
        ruleMap.put("z-index", new StyleRule("0") {
            @Override void modifyMesh(Mesh mesh, Window window, String value) throws Exception {
                mesh.position.z = Float.valueOf(value);
            }
        });
        ruleMap.put("position", new StyleRule("0 0") {
            @Override
            void modifyMesh(Mesh mesh, Window window, String value) throws Exception {
                String[] parts = value.split(" ");
                mesh.position.x = (Float.valueOf(parts[0]) / 100f) * window.getWidth();
                mesh.position.y = (Float.valueOf(parts[1]) / 100f) * window.getHeight();
            }
        });
        ruleMap.put("color", new StyleRule("0 0 0") {
            @Override
            void modifyMesh(Mesh mesh, Window window, String value) throws Exception {
                String[] parts = value.split(" ");

                mesh.material.ambientColour.set(Color3.valueOf(value));
                if (parts.length == 4) mesh.material.ambientColour.setA(Float.valueOf(parts[3]));
            }
        });
        ruleMap.put("scale", new StyleRule("1") {
            @Override
            void modifyMesh(Mesh mesh, Window window, String value) throws Exception {
                mesh.scale = Float.valueOf(value);
            }
        });
        ruleMap.put("rotation", new StyleRule("0") {
            @Override
            void modifyMesh(Mesh mesh, Window window, String value) throws Exception {
                mesh.rotation.z = (float) ((Math.PI * 2) * Float.valueOf(value));
            }
        });
        ruleMap.put("width", new StyleRule("0"));
        ruleMap.put("height", new StyleRule("0"));
        ruleMap.put("font", new StyleRule("$fonts/spaceAge.ttf"));
    }

    public StyleSet(String text) throws Exception {
        this.initRuleMap();

        String[] lines = text.replace("\n", "").split(";");

        for (String line : lines) {
            if (line.trim().equals("")) continue;

            String[] parts = line.split(":");
            parts[0] = parts[0].trim();
            parts[1] = parts[1].trim();

            if (ruleMap.containsKey(parts[0])) {
                StyleRule rule = ruleMap.get(parts[0]);
                rule.value = parts[1];
                rule.written = true;
            }
            else Logger.log("Failed to find style rule of name '" + parts[0] + "'", Logger.LogType.WARN);
        }
    }

    public StyleSet() {
        this.initRuleMap();
    }

    public StyleSet(Map<String, StyleRule> map) {
        for (String key : map.keySet()) {
            ruleMap.put(key, map.get(key).clone());
        }
    }

    public void overwrite(StyleSet styleSet) {
        for (String key : styleSet.ruleMap.keySet()) {
            StyleRule rule = styleSet.ruleMap.get(key);
            if (rule.written) this.ruleMap.get(key).set(rule);
        }
    }

    @Override
    public String toString() {
        String text = "";
        for (String key : this.ruleMap.keySet()) {
            StyleRule rule = this.ruleMap.get(key);
            text += key + ": " + rule.value + ": " + rule.written + "\n";
        }
        return text.trim();
    }

    public StyleSet clone() {
        return new StyleSet(this.ruleMap);
    }

    public static class StyleRule {
        public String value = null;
        public boolean written = false;

        public StyleRule() {

        }

        public StyleRule(String value) {
            this.value = value;
        }

        public void updateMesh(Mesh mesh, Window window) throws Exception {
            if (value != null) this.modifyMesh(mesh, window, this.value);
        }

        void modifyMesh(Mesh mesh, Window window, String value) throws Exception {

        }

        void set(StyleRule rule2) {
            this.value = rule2.value;
            this.written = rule2.written;
        }

        public void setValue(String value) {
            this.value = value;
            this.written = true;
        }
        public void setValue(Object value) {
            this.value = value.toString();
            this.written = true;
        }
        public void setValue(float value) {
            this.value = String.valueOf(value);
            this.written = true;
        }

        protected StyleRule clone() {
            return new StyleRule(this.value) {
                @Override
                void modifyMesh(Mesh mesh, Window window, String value) throws Exception {
                    StyleRule.this.modifyMesh(mesh, window, value);
                }
            };
        }
    }
}
package net.comsoria.engine.loaders.xhtml.ui;

import net.comsoria.engine.Scene;
import net.comsoria.engine.loaders.xml.XMLNode;
import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.Renderable;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.engine.view.graph.mesh.Mesh;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;

public abstract class UINode implements Renderable {
    public StyleSet styleSet;

    protected XMLNode originalXML;

    protected Mesh mesh;

    public List<Tuple<EventType, EventListener>> eventListeners = new ArrayList<>();

    public UINode(StyleSet styleSet, XMLNode xmlNode) {
        this.styleSet = styleSet;
        this.originalXML = xmlNode;
    }

    protected abstract void genMesh(Window window) throws Exception;

    public void updateStyleSets(Window window) throws Exception {
        if (this.mesh != null)
            for (String name : styleSet.ruleMap.keySet()) {
                styleSet.ruleMap.get(name).updateMesh(mesh, window);
            }
    }

    public Mesh getMesh() {
        return mesh;
    }

    public void addEventListener(EventType eventType, EventListener eventListener) {
        eventListeners.add(new Tuple<>(eventType, eventListener));
    }

    public void event(EventType type) {
        for (Tuple<EventType, EventListener> listenerTuple : eventListeners) {
            if (listenerTuple.getA() == type) listenerTuple.getB().call(this);
        }
    }

    @Override
    public Closeable render(Transformation transformation, Scene scene, RenderData renderData, Window window) throws Exception {
        return mesh.render(transformation, scene, renderData, window);
    }

    @Override
    public void cleanup() {
        if (this.mesh != null) mesh.cleanup();
    }

    @Override
    public boolean shouldRender() {
        return true;
    }

    public enum EventType {
        CLICK,
        HOVER_ON,
        HOVER_OFF
    }

    public interface EventListener {
        void call(UINode node);
    }
}
package net.comsoria.engine.loaders.xhtml;

import net.comsoria.engine.loaders.xhtml.ui.Document;
import net.comsoria.engine.loaders.xhtml.ui.UINode;
import net.comsoria.engine.loaders.xhtml.ui.StyleSet;
import net.comsoria.engine.loaders.xhtml.ui.node.Canvas;
import net.comsoria.engine.loaders.xhtml.ui.node.Model;
import net.comsoria.engine.loaders.xhtml.ui.node.Paragraph;
import net.comsoria.engine.loaders.xhtml.ui.node.Rectangle;
import net.comsoria.engine.loaders.xml.XMLLoader;
import net.comsoria.engine.loaders.xml.XMLNode;
import net.comsoria.engine.utils.Logger;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.color.Color3;

import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class XHTMLLoader {
    private static Map<String, Class<? extends UINode>> nodeTypes = new HashMap<>();

    static {
        nodeTypes.put("p", Paragraph.class);
        nodeTypes.put("canvas", Canvas.class);
        nodeTypes.put("rect", Rectangle.class);
        nodeTypes.put("model", Model.class);
    }

    public static Document loadDocument(String text, Window window) throws Exception {
        Document result = new Document();

        XMLNode page = XMLLoader.loadXML(text);

        Map<String, StyleSet> styles = new HashMap<>();

        String bg = page.getParam("bg");
        if (bg != null) result.background = Color3.valueOf(bg);

        for (XMLNode node : page.getAllChildren()) {
            if (node.getTagName().equals("style")) {
                styles.putAll(getStyleSets(node.getInnerXML()));
            } else if (nodeTypes.containsKey(node.getTagName())) {
                result.addNode(genNode(node.getTagName(), findStyleSet(styles, node), node), window);
            } else {
                Logger.log("Failed to find tag of type '" + node.getTagName() + "'", Logger.LogType.WARN);
            }
        }

        return result;
    }

    private static UINode genNode(String type, StyleSet styleSet, XMLNode xmlNode) throws Exception {
        return (UINode) nodeTypes.get(type).getConstructors()[0].newInstance(styleSet, xmlNode);
    }

    private static StyleSet findStyleSet(Map<String, StyleSet> existingStyles, XMLNode node) {
        StyleSet set = existingStyles.getOrDefault(node.getTagName(), StyleSet.defaultStyleSet).clone();

        String id = node.getParam("class");
        StyleSet set2 = existingStyles.getOrDefault("." + id, null);
        if (set2 != null) set.overwrite(set2);

        id = node.getParam("id");
        set2 = existingStyles.getOrDefault("#" + id, null);
        if (set2 != null) set.overwrite(set2);

        return set;
    }

    private static final Pattern blockStart = Pattern.compile("([a-zA-Z\\.,_0-9#]+) ?\\{([^}]*)\\}", Pattern.MULTILINE);

    private static Map<String, StyleSet> getStyleSets(String css) throws Exception {
        Map<String, StyleSet> result = new HashMap<>();

        Matcher matcher = blockStart.matcher(css);
        while (matcher.find()) {
            result.put(matcher.group(1), new StyleSet(matcher.group(2)));
        }

        return result;
    }
}
package net.comsoria.engine.loaders.xml;

import net.comsoria.engine.loaders.LoaderException;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class XMLLoader {
    private final static Pattern opening = Pattern.compile("<([A-Za-z0-9_]+)(( [^=^ ]+=\"[^\"]+\")*)>");
    private final static Pattern closing = Pattern.compile("</([A-Za-z0-9_]+)>");
    private final static Pattern element = Pattern.compile("<[^>]+>");
    private final static Pattern param = Pattern.compile("([^=^ ]+) ?= ?\"([^\"]+)\"");

    public static XMLNode loadXML(String xml) {
        Matcher elementMatcher = element.matcher(xml);

        List<GeneratedNode> found = new ArrayList<>();
        List<UnparentedNode> unparentedNodes = new ArrayList<>();

        int layer = 0;
        int lastIndex = 0;
        while (elementMatcher.find()) {
            String toAdd = xml.substring(lastIndex, elementMatcher.start());
            for (GeneratedNode generatedNode : found) {
                if (layer - 1 == generatedNode.layer) {
                    generatedNode.text += toAdd;
                }
            }

            lastIndex = elementMatcher.end();

            Matcher matcher = opening.matcher(elementMatcher.group(0));

            if (matcher.find()) { //Opening
                int others = 0;
                for (UnparentedNode unparentedNode : unparentedNodes) {
                    if (unparentedNode.layer == layer)
                        others += unparentedNode.node.toString().length();

                }

                found.add(new GeneratedNode(
                        matcher.group(1),
                        matcher.group(2),
                        layer,
                        found.size() == 0? 0:found.get(found.size() - 1).index,
                        lastIndex,
                        matcher.group().length(),
                        others
                ));

                layer += 1;
            } else {
                matcher = closing.matcher(elementMatcher.group(0));

                GeneratedNode last = found.get(found.size() - 1);
                if (matcher.find() && matcher.group(1).equals(last.tagName)) { //Closing
                    UnparentedNode unparentedNode = new UnparentedNode(
                            last.layer,
                            last.getNode()
                    );
                    found.remove(last);

                    for (UnparentedNode child : new ArrayList<>(unparentedNodes)) {
                        if (child.layer > last.layer) {
                            unparentedNode.node.children.add(child.node);
                            unparentedNodes.remove(child);
                        }
                    }

                    unparentedNodes.add(unparentedNode);

                    layer -= 1;
                } else { //Error
                    throw new LoaderException("Invalid syntax in XML around '" + elementMatcher.group(0) + "'");
                }
            }
        }

        return unparentedNodes.get(0).node;
    }

    private static class GeneratedNode {
        String tagName;
        String params;
        int layer;
        String text = "";
        int parentStart;
        private final int index;

        GeneratedNode(String tagName, String params, int layer, int lastIndex, int index, int size, int others) {
            this.tagName = tagName;
            this.params = params;
            this.layer = layer;
            this.parentStart = index - lastIndex - size - others;
            this.index = index;
        }

        XMLNode getNode() {
            XMLNode xmlNode = new XMLNode(this.tagName, text, parentStart, new ArrayList<>());

            Matcher paramMatcher = param.matcher(this.params);
            while (paramMatcher.find()) {
                xmlNode.parameters.add(new XMLNode.Parameter(paramMatcher.group(1), paramMatcher.group(2)));
            }

            return xmlNode;
        }
    }

    private static class UnparentedNode {
        int layer;
        XMLNode node;

        public UnparentedNode(int layer, XMLNode node) {
            this.layer = layer;
            this.node = node;
        }
    }
}
package net.comsoria.engine.loaders.xml;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class XMLNode {
    public List<Parameter> parameters = new ArrayList<>();
    private final String tagName;
    public String innerText;
    public List<XMLNode> children;
    public final int parentStart;

    public XMLNode(String tagName, String innerText, int parentStart, List<XMLNode> children) {
        this.tagName = tagName;
        this.children = children;
        this.innerText = innerText;
        this.parentStart = parentStart;
    }

    public String getTagName() {
        return tagName;
    }

    public String getInnerText() {
        StringBuilder result = new StringBuilder();
        result.append(innerText);

        int offset = 0;
        for (XMLNode node : children) {
            String nodeXML = node.getInnerText();
            result.insert(node.parentStart + offset, nodeXML);
            offset += nodeXML.length();
        }

        return result.toString();
    }

    public String getOpener() {
        String text = "<" + tagName;
        for (Parameter parameter : parameters) {
            text += " " + parameter.name + "=\"" + parameter.value + "\"";
        }

        return text + ">";
    }

    public String getCloser() {
        return "</" + tagName + ">";
    }

    public String getInnerXML() {
        StringBuilder result = new StringBuilder();
        result.append(innerText);

        int offset = 0;
        for (XMLNode node : children) {
            String nodeXML = node.getOuterXML();
            result.insert(node.parentStart + offset, nodeXML);
            offset += nodeXML.length();
        }

        return result.toString();
    }
    public String getOuterXML() {
        return this.getOpener() + this.getInnerXML() + this.getCloser();
    }

    @Override
    public String toString() {
        return this.getOuterXML();
    }

    public boolean hasParam(String name) {
        for (Parameter parameter : parameters) {
            if (parameter.name.equals(name)) return true;
        }
        return false;
    }

    public String getParam(String name) {
        for (Parameter parameter : parameters) {
            if (parameter.name.equals(name)) return parameter.value;
        }

        return null;
    }

    public List<XMLNode> getAllChildren() {
        List<XMLNode> children = new ArrayList<>();
        for (XMLNode child : this.children) {
            children.add(child);
            children.addAll(child.getAllChildren());
        }
        return children;
    }

    public static class Parameter {
        final String name;
        final String value;

        public Parameter(String name, String value) {
            this.name = name;
            this.value = value;
        }
    }
}
package net.comsoria.engine.math;

public class Circle {
    public float radius = 1;

    public Circle() {

    }

    public Circle(float r) {
        this.radius = r;
    }

    public Line getTangent(float angle) {
        Line result = new Line();
        result.gradient = (float) -Math.tan(angle);
        result.yIntercept = (float) (this.radius * (Math.cos(angle) + (result.gradient * -Math.sin(angle))));
        return result;
    }
}
package net.comsoria.engine.math;

import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import org.joml.Vector2f;

public class Line implements GLSLUniformBindable {
    public float gradient = 0;
    public float yIntercept = 0;

    public Line(float gradient, float yIntercept) {
        this.gradient = gradient;
        this.yIntercept = yIntercept;
    }

    public Line() {

    }

    public Line(Vector2f p1, Vector2f p2) {
        this.gradient = (p2.y - p1.y) / (p2.x - p1.x);
        this.yIntercept = p1.y - (this.gradient * p1.x);
    }

    public float get(float x) {
        return (x * gradient) + yIntercept;
    }

    public Vector2f intersection(Line line) {
        float x = (line.yIntercept - this.yIntercept) / (this.gradient - line.gradient);
        float y = (this.gradient * x) + this.yIntercept;

        return new Vector2f(x, y);
    }

    public Vector2f getVec2() {
        return new Vector2f(this.gradient, this.yIntercept);
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name, this.getVec2());
    }
}
package net.comsoria.engine.math;

public class Plane {
    public float a, b, c, d;

    public Plane() {
        this(0, 0, 0, 0);
    }

    public Plane(float a, float b, float c, float d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }
}
package net.comsoria.engine.math;

public class Rectangle {
    public int width;
    public int height;
    public int x;
    public int y;

    public Rectangle() {
        this(0, 0, 0, 0);
    }

    public Rectangle(int width, int height, int x, int y) {
        this.width = width;
        this.height = height;
        this.x = x;
        this.y = y;
    }

    public Rectangle(int width, int height) {
        this(width, height, 0, 0);
    }
}
package net.comsoria.engine.math;

import org.joml.Vector3f;

public class Sphere {
    private Circle circle1;
    private Circle circle2;

    public Sphere(float radius) {
        this.circle1 = new Circle(radius);
        this.circle2 = new Circle(radius);
    }

    public Plane tangent(Vector3f angle) {
        Line tangent1 = this.circle1.getTangent((float) Math.atan2(angle.x, angle.y));
        Line tangent2 = this.circle2.getTangent((float) Math.atan2(angle.x, angle.z));

        return new Plane();
    }
}
package net.comsoria.engine;

import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.engine.view.Camera;
import net.comsoria.engine.view.FadeFog;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.light.SceneLight;
import net.comsoria.engine.view.Renderable;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;

public class Scene {
    public final SceneLight light;
    private final List<Renderable> children;
    public FadeFog fog;
    public final Camera camera;
    public Sky sky;

    public Scene() {
        this.light = new SceneLight();
        this.children = new ArrayList<>();
        this.camera = new Camera();
        this.sky = new Sky(-0.5f, 0.25f, 0.15f);
    }

    public void cleanup() {
        for (Renderable gameItem : this.children) {
            gameItem.cleanup();
        }
    }

    public void render(Transformation transformation, Window window) throws Exception {
        List<Closeable> toClose = new ArrayList<>();

        List<Renderable> start = new ArrayList<>();
        List<Renderable> middle = new ArrayList<>();
        List<Renderable> end = new ArrayList<>();
        for (Renderable renderable : this.children)
            switch (renderable.getRenderOrder()) {
                case First:
                    start.add(renderable);
                    break;
                case End:
                    end.add(renderable);
                    break;
                default:
                    middle.add(renderable);
                    break;
            }

        List<Renderable> total = new ArrayList<>(start);
        total.addAll(middle);
        total.addAll(end);

        for (Renderable gameItem : total) {
            if (!gameItem.shouldRender()) continue;

            Closeable item = gameItem.render(transformation, this, RenderData.defaultRenderData, window);
            if (item != null && !toClose.contains(item)) toClose.add(item);
        }

        for (Closeable closeable : toClose) closeable.close();
    }

    public void add(Renderable gameObject) throws Exception {
        this.children.add(gameObject);
    }

    public void updateLight(float time) {
        this.sky.calcColors(time);
        this.light.ambientLight.set(this.sky.getAmbience());
        this.light.directionalLight.direction.set(this.sky.getSunDirection());
    }
}
package net.comsoria.engine;

import net.comsoria.engine.view.color.Color3;
import net.comsoria.engine.view.color.Color4;
import net.comsoria.engine.view.graph.mesh.SkyBox;
import org.joml.Vector3f;

public class Sky {
    private final static Color3 day = new Color3(102,150,186).getOneToZero();
    private final static Color3 night = new Color3(23, 32, 42).getOneToZero();
    private final static Color3 sunset = new Color3(230, 81, 0).getOneToZero();

    private final Color3 mainColor = Color3.WHITE.clone();
    private final Color3 secondColor = Color3.WHITE.clone();
    private float ambience = 1.5f;
    private final Vector3f sunDirection = new Vector3f();

    private final float dayStart;
    private final float sunsetTime;
    private final float sunsetBand;

    public Sky(float dayStart, float sunsetTime, float sunsetBand) {
        this.dayStart = dayStart;
        this.sunsetTime = sunsetTime;
        this.sunsetBand = sunsetBand;
    }

    public void calcColors(float time) {
        float cos = (float) Math.cos(time);
        float mod = (float) (time % (Math.PI * 2) / (Math.PI * 2));
        this.sunDirection.set((float) Math.sin(time), cos, 0);

        mainColor.set(night);
        ambience = 2f;
        if (cos > dayStart) {
            float nCos = cos - dayStart;

            mainColor.set(mainColor.mix(day, nCos));
            ambience += nCos * 0.1;
        }

        float dist = Math.abs(mod - sunsetTime);
        this.secondColor.set(mainColor.mix(sunset, 1f - Math.min(Math.max(dist * (1f / sunsetBand), 0), 1)));
    }

    public Color3 getMainColor() {
        return mainColor;
    }

    public Color3 getSecondColor() {
        return secondColor;
    }

    public float getAmbience() {
        return ambience;
    }

    public Vector3f getSunDirection() {
        return sunDirection;
    }
}
package net.comsoria.engine.utils;

import org.joml.Vector2f;
import org.joml.Vector2i;

import java.util.ArrayList;
import java.util.List;

import static java.lang.reflect.Array.newInstance;

public class Grid<T> {
    private final List<T> rawItems;

    private final int width;
    private final int height;

    public Grid(int width, int height) {
        this(width, height, null);
    }

    public Grid(int width, int height, T nullvalue) {
        this.width = width;
        this.height = height;

        this.rawItems = new ArrayList<>();
        for (int i = 0; i < this.length(); i++) {
            this.rawItems.add(nullvalue);
        }
    }

    public int getHeight() {
        return height;
    }
    public int getWidth() {
        return width;
    }

    public int getIndex(int x, int y) {
        return (this.width * y) + x;
    }

    public T get(int x, int y) {
        return this.rawItems.get(this.getIndex(x, y));
    }
    public void set(int x, int y, T object) {
        this.rawItems.set(this.getIndex(x, y), object);
    }

    public T get(int i) {
        return this.rawItems.get(i);
    }
    public void set(int i, T object) {
        this.rawItems.set(i, object);
    }

    public boolean contains(T object) {
        for (T item : this.rawItems) {
            if (item.equals(object))  return true;
        }
        return false;
    }

    public void clear() {
        this.rawItems.clear();
    }

    public void fill(T object) {
        for (int i = 0; i < this.rawItems.size(); i++) {
            this.rawItems.set(i, object);
        }
    }

    public T[] getArray(Class tClass) {
        @SuppressWarnings("unchecked")
        T[] array = (T[]) newInstance(tClass, this.rawItems.size());
        for (int i = 0; i < this.rawItems.size(); i++) {
            array[i] = this.rawItems.get(i);
        }
        return array;
    }

    public void foreach(IterationCallback callback) {
        for (int i = 0; i < this.rawItems.size(); i++) {
            if (callback.run(this.rawItems.get(i), i)) return;
        }
    }

    public int length() {
        return width * height;
    }

    @Override
    public String toString() {
        return "Grid[" + this.length() + "]";
    }

    public Vector2i getXY(int i) {
        int x = i % this.width;
        int y = (i - x) / this.height;
        return new Vector2i(x, y);
    }
}
package net.comsoria.engine.utils;

public interface IterationCallback {
    boolean run(Object object, int index);
}
package net.comsoria.engine.utils;

import net.comsoria.engine.loaders.FileLoader;
import org.json.JSONObject;

import java.io.File;
import java.io.IOException;

public class JSONFile extends JSONObject {
    private final String path;

    public JSONFile(File file) throws IOException {
        this(file.toString());
    }

    public JSONFile(String path) throws IOException {
        super(FileLoader.loadResourceAsString(path));

        this.path = path;
    }

    public void save() throws IOException {
        FileLoader.writeResource(path, this.toString(4));
    }
}
package net.comsoria.engine.utils;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Logger {
    private static MultiOutputStream outputStream = new MultiOutputStream(System.out);
    private static PrintStream stream = new PrintStream(outputStream);

    static {

    }

    public static void log(String msg) throws IOException {
        log(msg, LogType.INFO);
    }

    public static void log(String msg, LogType type) throws IOException {
        msg = "[" + type.toString() + ": " + new SimpleDateFormat("HH:mm:ss").format(new Date()) + "]: " + msg;

//        System.out.println(msg);
        stream.println(msg);
    }

    public static void addOutputStream(OutputStream stream) {
        Logger.outputStream.outputs.add(stream);
//        System.setOut(stream);
//        System.setErr(stream);
    }

    public static void logRaw(String msg) {
        stream.println(msg);
    }

    public enum LogType {
        INFO,
        WARN,
        ERROR
    }
}
package net.comsoria.engine.utils;

import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MultiOutputStream extends OutputStream {
    public List<OutputStream> outputs = new ArrayList<>();

    public MultiOutputStream(OutputStream... streams) {
        outputs.addAll(Arrays.asList(streams));
    }

    @Override public void write(int b) throws IOException {
        for (OutputStream outputStream : outputs) outputStream.write(b);
    }

    @Override public void write(byte[] b) throws IOException {
        for (OutputStream outputStream : outputs) outputStream.write(b);
    }

    @Override public void write(byte[] b, int off, int len) throws IOException {
        for (OutputStream outputStream : outputs) outputStream.write(b, off, len);
    }


    @Override
    public void flush() throws IOException {
        for (OutputStream outputStream : outputs) outputStream.flush();
    }

    @Override
    public void close() throws IOException {
        for (OutputStream outputStream : outputs) {
            try {
                outputStream.close();
            } catch (Exception e) {
                Logger.log(e.getLocalizedMessage(), Logger.LogType.ERROR);
            }
        }
    }


}
package net.comsoria.engine.utils.random;

public interface NoiseGenerator {
    float noise(float x, float y, float seed);
    float noise(float x, float y, float z, float seed);
}
package net.comsoria.engine.utils.random;

import org.joml.Vector2f;
import org.joml.Vector2i;
import org.joml.Vector3f;

public class Random implements NoiseGenerator {
    public static Random random = new Random();

    public float noise(float x, float y, float seed) {
        return (float) (((Math.abs(Math.sin((x * 172.192) + (y * 827.232)) * seed) % 1) - 0.5) * 2);
    }

    public float noise(Vector2f vector2f, float seed) {
        return noise(vector2f.x, vector2f.y, seed);
    }

    public float noise(Vector2i vector2i, float seed) {
        return noise(vector2i.x, vector2i.y, seed);
    }

    public float noise(float x, float y, float z, float seed) {
        return (float) (((Math.abs(Math.sin((x * 172.192) + (y * 827.232) + (z * 436.876)) * seed) % 1) - 0.5) * 2);
    }

    public float noise(Vector3f vector3f, float seed) {
        return noise(vector3f.x, vector3f.y, vector3f.z, seed);
    }
}
package net.comsoria.engine.utils;

/*
 * A speed-improved simplex noise algorithm for 2D, 3D and 4D in Java.
 *
 * Based on example code by Stefan Gustavson (stegu@itn.liu.se).
 * Optimisations by Peter Eastman (peastman@drizzle.stanford.edu).
 * Better rank ordering method for 4D by Stefan Gustavson in 2012.
 *
 * This could be speeded up even further, but it's useful as it is.
 *
 * Version 2012-03-09
 *
 * This code was placed in the public domain by its original author,
 * Stefan Gustavson. You may use it as you see fit, but
 * attribution is appreciated.
 *
 */


public class SimplexNoise {  // Simplex noise in 2D, 3D and 4D
    private static Grad grad3[] = {new Grad(1,1,0),new Grad(-1,1,0),new Grad(1,-1,0),new Grad(-1,-1,0),
            new Grad(1,0,1),new Grad(-1,0,1),new Grad(1,0,-1),new Grad(-1,0,-1),
            new Grad(0,1,1),new Grad(0,-1,1),new Grad(0,1,-1),new Grad(0,-1,-1)};

    private static Grad grad4[]= {new Grad(0,1,1,1),new Grad(0,1,1,-1),new Grad(0,1,-1,1),new Grad(0,1,-1,-1),
            new Grad(0,-1,1,1),new Grad(0,-1,1,-1),new Grad(0,-1,-1,1),new Grad(0,-1,-1,-1),
            new Grad(1,0,1,1),new Grad(1,0,1,-1),new Grad(1,0,-1,1),new Grad(1,0,-1,-1),
            new Grad(-1,0,1,1),new Grad(-1,0,1,-1),new Grad(-1,0,-1,1),new Grad(-1,0,-1,-1),
            new Grad(1,1,0,1),new Grad(1,1,0,-1),new Grad(1,-1,0,1),new Grad(1,-1,0,-1),
            new Grad(-1,1,0,1),new Grad(-1,1,0,-1),new Grad(-1,-1,0,1),new Grad(-1,-1,0,-1),
            new Grad(1,1,1,0),new Grad(1,1,-1,0),new Grad(1,-1,1,0),new Grad(1,-1,-1,0),
            new Grad(-1,1,1,0),new Grad(-1,1,-1,0),new Grad(-1,-1,1,0),new Grad(-1,-1,-1,0)};

    private static short p[] = {151,160,137,91,90,15,
            131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
            190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
            88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
            77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
            102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
            135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
            5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
            223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
            129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
            251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
            49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
            138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180};
    // To remove the need for index wrapping, double the permutation table length
    private static short perm[] = new short[512];
    private static short permMod12[] = new short[512];
    static {
        for(int i=0; i<512; i++)
        {
            perm[i]=p[i & 255];
            permMod12[i] = (short)(perm[i] % 12);
        }
    }

    // Skewing and unskewing factors for 2, 3, and 4 dimensions
    private static final double F2 = 0.5*(Math.sqrt(3.0)-1.0);
    private static final double G2 = (3.0-Math.sqrt(3.0))/6.0;
    private static final double F3 = 1.0/3.0;
    private static final double G3 = 1.0/6.0;
    private static final double F4 = (Math.sqrt(5.0)-1.0)/4.0;
    private static final double G4 = (5.0-Math.sqrt(5.0))/20.0;

    // This method is a *lot* faster than using (int)Math.floor(x)
    private static int fastfloor(double x) {
        int xi = (int)x;
        return x<xi ? xi-1 : xi;
    }

    private static double dot(Grad g, double x, double y) {
        return g.x*x + g.y*y; }

    private static double dot(Grad g, double x, double y, double z) {
        return g.x*x + g.y*y + g.z*z; }

    private static double dot(Grad g, double x, double y, double z, double w) {
        return g.x*x + g.y*y + g.z*z + g.w*w; }


    // 2D simplex noise
    public static double noise(double xin, double yin) {
        double n0, n1, n2; // Noise contributions from the three corners
        // Skew the input space to determine which simplex cell we're in
        double s = (xin+yin)*F2; // Hairy factor for 2D
        int i = fastfloor(xin+s);
        int j = fastfloor(yin+s);
        double t = (i+j)*G2;
        double X0 = i-t; // Unskew the cell origin back to (x,y) space
        double Y0 = j-t;
        double x0 = xin-X0; // The x,y distances from the cell origin
        double y0 = yin-Y0;
        // For the 2D case, the simplex shape is an equilateral triangle.
        // Determine which simplex we are in.
        int i1, j1; // Offsets for second (middle) corner of simplex in (i,j) coords
        if(x0>y0) {i1=1; j1=0;} // lower triangle, XY order: (0,0)->(1,0)->(1,1)
        else {i1=0; j1=1;}      // upper triangle, YX order: (0,0)->(0,1)->(1,1)
        // A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
        // a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
        // c = (3-sqrt(3))/6
        double x1 = x0 - i1 + G2; // Offsets for middle corner in (x,y) unskewed coords
        double y1 = y0 - j1 + G2;
        double x2 = x0 - 1.0 + 2.0 * G2; // Offsets for last corner in (x,y) unskewed coords
        double y2 = y0 - 1.0 + 2.0 * G2;
        // Work out the hashed gradient indices of the three simplex corners
        int ii = i & 255;
        int jj = j & 255;
        int gi0 = permMod12[ii+perm[jj]];
        int gi1 = permMod12[ii+i1+perm[jj+j1]];
        int gi2 = permMod12[ii+1+perm[jj+1]];
        // Calculate the contribution from the three corners
        double t0 = 0.5 - x0*x0-y0*y0;
        if(t0<0) n0 = 0.0;
        else {
            t0 *= t0;
            n0 = t0 * t0 * dot(grad3[gi0], x0, y0);  // (x,y) of grad3 used for 2D gradient
        }
        double t1 = 0.5 - x1*x1-y1*y1;
        if(t1<0) n1 = 0.0;
        else {
            t1 *= t1;
            n1 = t1 * t1 * dot(grad3[gi1], x1, y1);
        }
        double t2 = 0.5 - x2*x2-y2*y2;
        if(t2<0) n2 = 0.0;
        else {
            t2 *= t2;
            n2 = t2 * t2 * dot(grad3[gi2], x2, y2);
        }
        // Add contributions from each corner to get the final noise value.
        // The result is scaled to return values in the interval [-1,1].
        return 70.0 * (n0 + n1 + n2);
    }


    // 3D simplex noise
    public static double noise(double xin, double yin, double zin) {
        double n0, n1, n2, n3; // Noise contributions from the four corners
        // Skew the input space to determine which simplex cell we're in
        double s = (xin+yin+zin)*F3; // Very nice and simple skew factor for 3D
        int i = fastfloor(xin+s);
        int j = fastfloor(yin+s);
        int k = fastfloor(zin+s);
        double t = (i+j+k)*G3;
        double X0 = i-t; // Unskew the cell origin back to (x,y,z) space
        double Y0 = j-t;
        double Z0 = k-t;
        double x0 = xin-X0; // The x,y,z distances from the cell origin
        double y0 = yin-Y0;
        double z0 = zin-Z0;
        // For the 3D case, the simplex shape is a slightly irregular tetrahedron.
        // Determine which simplex we are in.
        int i1, j1, k1; // Offsets for second corner of simplex in (i,j,k) coords
        int i2, j2, k2; // Offsets for third corner of simplex in (i,j,k) coords
        if(x0>=y0) {
            if(y0>=z0)
            { i1=1; j1=0; k1=0; i2=1; j2=1; k2=0; } // X Y Z order
            else if(x0>=z0) { i1=1; j1=0; k1=0; i2=1; j2=0; k2=1; } // X Z Y order
            else { i1=0; j1=0; k1=1; i2=1; j2=0; k2=1; } // Z X Y order
        }
        else { // x0<y0
            if(y0<z0) { i1=0; j1=0; k1=1; i2=0; j2=1; k2=1; } // Z Y X order
            else if(x0<z0) { i1=0; j1=1; k1=0; i2=0; j2=1; k2=1; } // Y Z X order
            else { i1=0; j1=1; k1=0; i2=1; j2=1; k2=0; } // Y X Z order
        }
        // A step of (1,0,0) in (i,j,k) means a step of (1-c,-c,-c) in (x,y,z),
        // a step of (0,1,0) in (i,j,k) means a step of (-c,1-c,-c) in (x,y,z), and
        // a step of (0,0,1) in (i,j,k) means a step of (-c,-c,1-c) in (x,y,z), where
        // c = 1/6.
        double x1 = x0 - i1 + G3; // Offsets for second corner in (x,y,z) coords
        double y1 = y0 - j1 + G3;
        double z1 = z0 - k1 + G3;
        double x2 = x0 - i2 + 2.0*G3; // Offsets for third corner in (x,y,z) coords
        double y2 = y0 - j2 + 2.0*G3;
        double z2 = z0 - k2 + 2.0*G3;
        double x3 = x0 - 1.0 + 3.0*G3; // Offsets for last corner in (x,y,z) coords
        double y3 = y0 - 1.0 + 3.0*G3;
        double z3 = z0 - 1.0 + 3.0*G3;
        // Work out the hashed gradient indices of the four simplex corners
        int ii = i & 255;
        int jj = j & 255;
        int kk = k & 255;
        int gi0 = permMod12[ii+perm[jj+perm[kk]]];
        int gi1 = permMod12[ii+i1+perm[jj+j1+perm[kk+k1]]];
        int gi2 = permMod12[ii+i2+perm[jj+j2+perm[kk+k2]]];
        int gi3 = permMod12[ii+1+perm[jj+1+perm[kk+1]]];
        // Calculate the contribution from the four corners
        double t0 = 0.6 - x0*x0 - y0*y0 - z0*z0;
        if(t0<0) n0 = 0.0;
        else {
            t0 *= t0;
            n0 = t0 * t0 * dot(grad3[gi0], x0, y0, z0);
        }
        double t1 = 0.6 - x1*x1 - y1*y1 - z1*z1;
        if(t1<0) n1 = 0.0;
        else {
            t1 *= t1;
            n1 = t1 * t1 * dot(grad3[gi1], x1, y1, z1);
        }
        double t2 = 0.6 - x2*x2 - y2*y2 - z2*z2;
        if(t2<0) n2 = 0.0;
        else {
            t2 *= t2;
            n2 = t2 * t2 * dot(grad3[gi2], x2, y2, z2);
        }
        double t3 = 0.6 - x3*x3 - y3*y3 - z3*z3;
        if(t3<0) n3 = 0.0;
        else {
            t3 *= t3;
            n3 = t3 * t3 * dot(grad3[gi3], x3, y3, z3);
        }
        // Add contributions from each corner to get the final noise value.
        // The result is scaled to stay just inside [-1,1]
        return 32.0*(n0 + n1 + n2 + n3);
    }


    // 4D simplex noise, better simplex rank ordering method 2012-03-09
    public static double noise(double x, double y, double z, double w) {

        double n0, n1, n2, n3, n4; // Noise contributions from the five corners
        // Skew the (x,y,z,w) space to determine which cell of 24 simplices we're in
        double s = (x + y + z + w) * F4; // Factor for 4D skewing
        int i = fastfloor(x + s);
        int j = fastfloor(y + s);
        int k = fastfloor(z + s);
        int l = fastfloor(w + s);
        double t = (i + j + k + l) * G4; // Factor for 4D unskewing
        double X0 = i - t; // Unskew the cell origin back to (x,y,z,w) space
        double Y0 = j - t;
        double Z0 = k - t;
        double W0 = l - t;
        double x0 = x - X0;  // The x,y,z,w distances from the cell origin
        double y0 = y - Y0;
        double z0 = z - Z0;
        double w0 = w - W0;
        // For the 4D case, the simplex is a 4D shape I won't even try to describe.
        // To find out which of the 24 possible simplices we're in, we need to
        // determine the magnitude ordering of x0, y0, z0 and w0.
        // Six pair-wise comparisons are performed between each possible pair
        // of the four coordinates, and the results are used to rank the numbers.
        int rankx = 0;
        int ranky = 0;
        int rankz = 0;
        int rankw = 0;
        if(x0 > y0) rankx++; else ranky++;
        if(x0 > z0) rankx++; else rankz++;
        if(x0 > w0) rankx++; else rankw++;
        if(y0 > z0) ranky++; else rankz++;
        if(y0 > w0) ranky++; else rankw++;
        if(z0 > w0) rankz++; else rankw++;
        int i1, j1, k1, l1; // The integer offsets for the second simplex corner
        int i2, j2, k2, l2; // The integer offsets for the third simplex corner
        int i3, j3, k3, l3; // The integer offsets for the fourth simplex corner
        // [rankx, ranky, rankz, rankw] is a 4-vector with the numbers 0, 1, 2 and 3
        // in some order. We use a thresholding to set the coordinates in turn.
        // Rank 3 denotes the largest coordinate.
        i1 = rankx >= 3 ? 1 : 0;
        j1 = ranky >= 3 ? 1 : 0;
        k1 = rankz >= 3 ? 1 : 0;
        l1 = rankw >= 3 ? 1 : 0;
        // Rank 2 denotes the second largest coordinate.
        i2 = rankx >= 2 ? 1 : 0;
        j2 = ranky >= 2 ? 1 : 0;
        k2 = rankz >= 2 ? 1 : 0;
        l2 = rankw >= 2 ? 1 : 0;
        // Rank 1 denotes the second smallest coordinate.
        i3 = rankx >= 1 ? 1 : 0;
        j3 = ranky >= 1 ? 1 : 0;
        k3 = rankz >= 1 ? 1 : 0;
        l3 = rankw >= 1 ? 1 : 0;
        // The fifth corner has all coordinate offsets = 1, so no need to compute that.
        double x1 = x0 - i1 + G4; // Offsets for second corner in (x,y,z,w) coords
        double y1 = y0 - j1 + G4;
        double z1 = z0 - k1 + G4;
        double w1 = w0 - l1 + G4;
        double x2 = x0 - i2 + 2.0*G4; // Offsets for third corner in (x,y,z,w) coords
        double y2 = y0 - j2 + 2.0*G4;
        double z2 = z0 - k2 + 2.0*G4;
        double w2 = w0 - l2 + 2.0*G4;
        double x3 = x0 - i3 + 3.0*G4; // Offsets for fourth corner in (x,y,z,w) coords
        double y3 = y0 - j3 + 3.0*G4;
        double z3 = z0 - k3 + 3.0*G4;
        double w3 = w0 - l3 + 3.0*G4;
        double x4 = x0 - 1.0 + 4.0*G4; // Offsets for last corner in (x,y,z,w) coords
        double y4 = y0 - 1.0 + 4.0*G4;
        double z4 = z0 - 1.0 + 4.0*G4;
        double w4 = w0 - 1.0 + 4.0*G4;
        // Work out the hashed gradient indices of the five simplex corners
        int ii = i & 255;
        int jj = j & 255;
        int kk = k & 255;
        int ll = l & 255;
        int gi0 = perm[ii+perm[jj+perm[kk+perm[ll]]]] % 32;
        int gi1 = perm[ii+i1+perm[jj+j1+perm[kk+k1+perm[ll+l1]]]] % 32;
        int gi2 = perm[ii+i2+perm[jj+j2+perm[kk+k2+perm[ll+l2]]]] % 32;
        int gi3 = perm[ii+i3+perm[jj+j3+perm[kk+k3+perm[ll+l3]]]] % 32;
        int gi4 = perm[ii+1+perm[jj+1+perm[kk+1+perm[ll+1]]]] % 32;
        // Calculate the contribution from the five corners
        double t0 = 0.6 - x0*x0 - y0*y0 - z0*z0 - w0*w0;
        if(t0<0) n0 = 0.0;
        else {
            t0 *= t0;
            n0 = t0 * t0 * dot(grad4[gi0], x0, y0, z0, w0);
        }
        double t1 = 0.6 - x1*x1 - y1*y1 - z1*z1 - w1*w1;
        if(t1<0) n1 = 0.0;
        else {
            t1 *= t1;
            n1 = t1 * t1 * dot(grad4[gi1], x1, y1, z1, w1);
        }
        double t2 = 0.6 - x2*x2 - y2*y2 - z2*z2 - w2*w2;
        if(t2<0) n2 = 0.0;
        else {
            t2 *= t2;
            n2 = t2 * t2 * dot(grad4[gi2], x2, y2, z2, w2);
        }
        double t3 = 0.6 - x3*x3 - y3*y3 - z3*z3 - w3*w3;
        if(t3<0) n3 = 0.0;
        else {
            t3 *= t3;
            n3 = t3 * t3 * dot(grad4[gi3], x3, y3, z3, w3);
        }
        double t4 = 0.6 - x4*x4 - y4*y4 - z4*z4 - w4*w4;
        if(t4<0) n4 = 0.0;
        else {
            t4 *= t4;
            n4 = t4 * t4 * dot(grad4[gi4], x4, y4, z4, w4);
        }
        // Sum up and scale the result to cover the range [-1,1]
        return 27.0 * (n0 + n1 + n2 + n3 + n4);
    }

    // Inner class to speed upp gradient computations
    // (In Java, array access is a lot slower than member access)
    private static class Grad
    {
        double x, y, z, w;

        Grad(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        Grad(double x, double y, double z, double w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }
    }
}
package net.comsoria.engine.utils;

public final class Timer {
    private static int gameLoopIndex = 0;

    public static long getTime() {
        return System.currentTimeMillis();
    }

    public static int getGameLoopIndex() {
        return gameLoopIndex;
    }

    public static void update() {
        gameLoopIndex += 1;
    }

    public static void update(int inc) {
        gameLoopIndex += inc;
    }
}
package net.comsoria.engine.utils;

public class Tuple<A,B> {
    public A a;
    public B b;

    public Tuple(A a, B b) {
        this.a = a;
        this.b = b;
    }

    public Tuple() {
        this.a = null;
        this.b = null;
    }

    public A getA() {
        return a;
    }

    public B getB() {
        return b;
    }

    public void setB(B b) {
        this.b = b;
    }

    public void setA(A a) {
        this.a = a;
    }
}
package net.comsoria.engine.utils;

import org.joml.*;
import org.lwjgl.BufferUtils;

import java.io.*;
import java.lang.Math;
import java.nio.ByteBuffer;
import java.util.*;

public class Utils {
    public static final Utils utils = new Utils();

    private Map<String, String> names = new HashMap<>();
    public JSONFile settings;

    public String getPath(String path) {
        for (String name : names.keySet()) {
            path = path.replace(name, names.get(name));
        }
        return path.replace("/", File.separator);
    }

    public void addName(String name, String path) {
        names.put("$" + name, getPath(path));
    }

    public void addName(String name, String path, boolean raw) {
        if (raw) names.put("$" + name, path);
        else addName(name, path);
    }

    public String p(String path) {
        return getPath(path);
    }

    public void createDirs(String[] paths) {
        for (String path : paths) {
            File file = new File(p(path));
            if (!file.exists()) file.mkdir();
        }
    }


    public final static String charset = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";

    public static String UUID() {
        StringJoiner result = new StringJoiner("-");

        int parts = (int) ((Math.random() * 3) + 4);
        for (int i = 0; i < parts; i++) {
            int chars = (int) ((Math.random() * 5) + 10);
            StringBuilder part = new StringBuilder();
            for (int c = 0; c < chars; c++) {
                part.append(charset.charAt((int) (Math.random() * charset.length())));
            }
            result.add(part.toString());
        }

        return result.toString();
    }


    public static List<Integer> toIntList(int[] input) {
        List<Integer> list = new ArrayList<>();
        for (int no : input) {
            list.add(no);
        }
        return list;
    }

    public static float[] listToArray(List<Float> input) {
        float[] result = new float[input.size()];
        for (int i = 0; i < input.size(); i++) result[i] = input.get(i);
        return result;
    }

    public static float map(float startMin, float startMax, float endMin, float endMax, float number) {
        float pc = (number - startMin) / (startMax - startMin);
        return ((endMax - endMin) * pc) + endMin;
    }

    public static ByteBuffer resizeBuffer(ByteBuffer buffer, int newCapacity) {
        ByteBuffer newBuffer = BufferUtils.createByteBuffer(newCapacity);
        buffer.flip();
        newBuffer.put(buffer);
        return newBuffer;
    }

    public static <K,V> HashMap<K, V> buildMap(Object... input) {
        HashMap<K, V> map = new HashMap<>();
        for (int i = 0; i < input.length; i += 2) {
            map.put((K) input[i], (V) input[i + 1]);
        }

        return map;
    }

    public static void println(PrintStream stream, float[] data) {
        for (Object object : data) {
            stream.print(object + " ");
        }
        stream.print('\n');
    }

    public static void println(PrintStream stream, Object... objects) {
        for (Object object : objects) {
            stream.print(object + " ");
        }
        stream.print('\n');
    }

    public static <T> T last(List<T> items) {
        return items.get(items.size() - 1);
    }

    public static float round(float value, int precision) {
        int scale = (int) Math.pow(10, precision);
        return (float) Math.round(value * scale) / scale;
    }

    public static Vector2f round(Vector2f value, int precision) {
        return new Vector2f(Utils.round(value.x, precision), Utils.round(value.y, precision));
    }

    public static float distance(float... coordinates) {
        float added = 0;

        for (float coordItem : coordinates) {
            added += coordItem * coordItem;
        }

        return (float) Math.sqrt(added);
    }

    public static String getString(Vector3f vec) {
        return "Vector3f[" + vec.x + "," + vec.y + "," + vec.z + "]";
    }

    public static String getString(Vector2f vec) {
        return "Vector2f[" + vec.x + "," + vec.y + "]";
    }

    public static String getString(Vector2i vec) {
        return "Vector2i[" + vec.x + "," + vec.y + "]";
    }

    public static String getString(Vector3i vec) {
        return "Vector3i[" + vec.x + "," + vec.y + "," + vec.z + "]";
    }
}
package net.comsoria.engine.view;

public class Animation {
    private TickHandler tickHandler;

    public Animation(TickHandler handler) {
        this.tickHandler = handler;
    }

    public void tick(float index) {
        tickHandler.update(index);
    }

    public interface TickHandler {
        void update(float index);
    }
}
package net.comsoria.engine.view.batch;

import java.io.Closeable;
import java.io.IOException;
import java.util.List;

public class BatchCloseable implements Closeable {
    private final List<Closeable> toClose;

    public BatchCloseable(List<Closeable> toClose) {
        this.toClose = toClose;
    }

    @Override public void close() throws IOException {
        for (Closeable closeable : toClose) closeable.close();
    }
}
package net.comsoria.engine.view.batch;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.Renderable;
import net.comsoria.engine.view.Window;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;

public class BatchRenderer implements Renderable {
    public final List<Renderable> gameObjects;
    public BatchRenderType batchRenderType;

    public boolean shouldRender = true;
    public RenderOrder renderPosition = RenderOrder.Any;

    public BatchRenderer(BatchRenderType batchRenderType, List<Renderable> gameObjects) {
        this.gameObjects = gameObjects;
        this.batchRenderType = batchRenderType;
    }

    public BatchRenderer(BatchRenderType batchRenderType) {
        this(batchRenderType, new ArrayList<>());
    }

    @Override
    public Closeable render(Transformation transformation, Scene scene, RenderData renderData, Window window) throws Exception {
        List<Closeable> toClose = new ArrayList<>();

        RenderData meshRenderData = new RenderData() {
            @Override public boolean shouldBindOwnGeometry() {
                return batchRenderType.geometry == null;
            }
            @Override public boolean shouldBindOwnShaderProgram() {
                return batchRenderType.shaderProgram == null;
            }
        };

        if (!meshRenderData.shouldBindOwnGeometry()) batchRenderType.geometry.bind();
        if (!meshRenderData.shouldBindOwnShaderProgram()) batchRenderType.shaderProgram.bind();

        List<Renderable> start = new ArrayList<>();
        List<Renderable> middle = new ArrayList<>();
        List<Renderable> end = new ArrayList<>();
        for (Renderable renderable : gameObjects)
            switch (renderable.getRenderOrder()) {
                case First:
                    start.add(renderable);
                    break;
                case End:
                    end.add(renderable);
                    break;
                default:
                    middle.add(renderable);
                    break;
            }

        List<Renderable> total = new ArrayList<>(start);
        total.addAll(middle);
        total.addAll(end);

        for (Renderable gameItem : total) {
            if (!gameItem.shouldRender()) continue;

            Closeable item = gameItem.render(transformation, scene, meshRenderData, window);
            if (item != null && !toClose.contains(item)) toClose.add(item);
        }

        for (Closeable closeable : toClose) closeable.close();

        if (!meshRenderData.shouldBindOwnGeometry()) batchRenderType.geometry.unbind();
        if (!meshRenderData.shouldBindOwnShaderProgram()) batchRenderType.shaderProgram.unbind();

        return new BatchCloseable(toClose);
    }

    @Override public void cleanup() {
        for (Renderable renderable : gameObjects) renderable.cleanup();
    }

    @Override
    public boolean shouldRender() {
        return shouldRender;
    }

    @Override
    public RenderOrder getRenderOrder() {
        return this.renderPosition;
    }
}
package net.comsoria.engine.view.batch;

import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.graph.Geometry;

public class BatchRenderType {
    public Geometry geometry = null;
    public ShaderProgram shaderProgram = null;

    public BatchRenderType() {

    }

    public BatchRenderType(Geometry geometry) {
        this.geometry = geometry;
    }

    public BatchRenderType(ShaderProgram shaderProgram) {
        this.shaderProgram = shaderProgram;
    }

    public BatchRenderType(Geometry geometry, ShaderProgram shaderProgram) {
        this.geometry = geometry;
        this.shaderProgram = shaderProgram;
    }
}
package net.comsoria.engine.view.batch;

public abstract class RenderData {
    public final static RenderData defaultRenderData = new RenderData() {
        @Override public boolean shouldBindOwnGeometry() {
            return true;
        }
        @Override public boolean shouldBindOwnShaderProgram() {
            return true;
        }
    };

    public abstract boolean shouldBindOwnGeometry();
    public abstract boolean shouldBindOwnShaderProgram();
}
package net.comsoria.engine.view;

import org.joml.Vector3f;

import java.awt.*;

public class Camera {
    public final Vector3f position;
    public final Vector3f rotation;

    public float fov = (float) Math.toRadians(80);
    public float near = 0.01f;
    public float far = 4000f;

    public Camera() {
        position = new Vector3f(0, 0, 0);
        rotation = new Vector3f(0, 0, 0);
    }

    public Camera(Vector3f position, Vector3f rotation) {
        this.position = position;
        this.rotation = rotation;
    }

    public void movePosition(float offsetX, float offsetY, float offsetZ) {
        if (offsetZ != 0) {
            position.x += (float) Math.sin(Math.toRadians(rotation.y)) * -1.0f * offsetZ;
            position.z += (float) Math.cos(Math.toRadians(rotation.y)) * offsetZ;
        }
        if (offsetX != 0) {
            position.x += (float) Math.sin(Math.toRadians(rotation.y - 90)) * -1.0f * offsetX;
            position.z += (float) Math.cos(Math.toRadians(rotation.y - 90)) * offsetX;
        }
        position.y += offsetY;
    }

    public void moveRotation(float offsetX, float offsetY, float offsetZ) {
        rotation.x += offsetX;
        rotation.y += offsetY;
        rotation.z += offsetZ;
    }
}
package net.comsoria.engine.view.color;

import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import org.joml.Vector3f;
import org.joml.Vector4f;

public class Color3 implements GLSLUniformBindable {
    public final static Color3 WHITE = new Color3(1);
    public final static Color3 BLACK = new Color3(0);
    public final static Color3 GRAY = new Color3(0.5f);
    public final static Color3 RED = new Color3(1, 0, 0);
    public final static Color3 BLUE = new Color3(0, 0, 1);
    public final static Color3 GREEN = new Color3(0, 1, 0);

    public float r;
    public float g;
    public float b;

    public Color3() {
        this(0, 0, 0);
    }

    public Color3(float r, float g, float b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public Color3(float value) {
        this(value, value, value);
    }

    public Color3(Vector3f vec) {
        this(vec.x, vec.y, vec.z);
    }

    public Color3(Vector4f vec) {
        this(vec.x, vec.y, vec.z);
    }

    public Color3(Color3 color) {
        this(color.r, color.g, color.b);
    }

    public float getR() {
        return r;
    }
    public float getG() {
        return g;
    }
    public float getB() {
        return b;
    }

    public Color3 setR(float r) {
        this.r = r;
        return this;
    }
    public Color3 setG(float g) {
        this.g = g;
        return this;
    }
    public Color3 setB(float b) {
        this.b = b;
        return this;
    }

    public Color3 getOneToZero() {
        return new Color4(this.r / 255f, this.g / 255f, this.b / 255f);
    }

    public Vector3f getVec3() {
        return new Vector3f(this.r, this.g, this.b);
    }

    public Color3 mix(Color3 color2, float dist) {
        float x = this.r + ((color2.r - this.r) * dist);
        float y = this.g + ((color2.g - this.g) * dist);
        float z = this.b + ((color2.b - this.b) * dist);
        return new Color3(x, y, z);
    }

    public Color3 clone() {
        return new Color3(this);
    }

    public Color3 set(float r, float g, float b) {
        this.r = r;
        this.g = g;
        this.b = b;

        return this;
    }

    public Color3 set(float x) {
        this.r = x;
        this.g = x;
        this.b = x;

        return this;
    }

    public Color3 set(Color3 color) {
        this.r = color.r;
        this.g = color.g;
        this.b = color.b;

        return this;
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name, this.getVec3());
    }

    public static Color3 valueOf(String string) {
        String[] parts = string.split(" ");

        return new Color3(Float.valueOf(parts[0]), Float.valueOf(parts[1]), Float.valueOf(parts[2]));
    }

    public String toString(boolean full) {
        if (full)
            return "Color3[" + this.r + "," + this.g + "," + this.b + "]";
        else
            return this.r + " " + this.g + " " + this.b;
    }

    @Override
    public String toString() {
        return this.toString(true);
    }
}
package net.comsoria.engine.view.color;

import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import org.joml.Vector3f;
import org.joml.Vector4f;

public class Color4 extends Color3 {
    public final static Color4 WHITE = new Color4(1);
    public final static Color4 BLACK = new Color4(0);
    public final static Color4 GRAY = new Color4(0.5f);
    public final static Color4 RED = new Color4(1, 0, 0);
    public final static Color4 BLUE = new Color4(0, 0, 1);
    public final static Color4 GREEN = new Color4(0, 1, 0);
    public final static Color4 TRANSPARENT = new Color4(0, 0, 0, 0);

    public float a = 1;

    public Color4() {
        this(0, 0, 0, 1f);
    }

    public Color4(float r, float g, float b, float a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    public Color4(float value) {
        this(value, value, value);
    }

    public Color4(float x, float y, float z) {
        this(x, y, z, 1f);
    }

    public Color4(Vector3f vec) {
        this(vec.x, vec.y, vec.z, 1f);
    }

    public Color4(Vector4f vec) {
        this(vec.x, vec.y, vec.z, vec.w);
    }

    public Color4(Color4 color) {
        this(color.r, color.g, color.b, color.a);
    }

    public float getR() {
        return r;
    }
    public float getG() {
        return g;
    }
    public float getB() {
        return b;
    }
    public float getA() {
        return a;
    }

    public Color4 setA(float a) {
        this.a = a;
        return this;
    }
    public Color4 setR(float r) {
        this.r = r;
        return this;
    }
    public Color4 setG(float g) {
        this.g = g;
        return this;
    }
    public Color4 setB(float b) {
        this.b = b;
        return this;
    }

    public Color4 getOneToZero() {
        return new Color4(this.r / 255f, this.g / 255f, this.b / 255f, this.a);
    }

    public Vector4f getVec4() {
        return new Vector4f(this.r, this.g, this.b, this.a);
    }

    public Color4 mix(Color4 color2, float dist) {
        float x = this.r + ((color2.r - this.r) * dist);
        float y = this.g + ((color2.g - this.g) * dist);
        float z = this.b + ((color2.b - this.b) * dist);
        float w = this.a + ((color2.a - this.a) * dist);
        return new Color4(x, y, z, w);
    }

    public Color4 clone() {
        return new Color4(this);
    }

    public Color4 set(float r, float g, float b, float a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;

        return this;
    }

    public Color4 set(float x) {
        this.r = x;
        this.g = x;
        this.a = x;

        return this;
    }

    public Color4 set(Color4 color) {
        this.r = color.r;
        this.g = color.g;
        this.b = color.b;
        this.a = color.a;

        return this;
    }

    public Color4 set(float r, float g, float b) {
        return this.set(r, g, b, this.a);
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name, this.getVec4());
    }

    public String toString(boolean full) {
        if (full)
            return "Color3[" + this.r + "," + this.g + "," + this.b + "," + this.a + "]";
        else
            return this.r + " " + this.g + " " + this.b + " " + this.a;
    }

    @Override
    public String toString() {
        return this.toString(true);
    }
}
package net.comsoria.engine.view;

import net.comsoria.engine.view.GLSL.GLSLException;
import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import org.joml.Vector3f;

import static org.lwjgl.opengl.GL11.*;

public class FadeFog implements GLSLUniformBindable {
    public float density;
    public float start;

    public FadeFog() {
        this.density = 0;
        this.start = 0;
    }

    public FadeFog(float density, float start) {
        this.density = density;
        this.start = start;
    }

    public static void create(ShaderProgram shaderProgram, String name) {
        shaderProgram.createUniform(name + ".density");
        shaderProgram.createUniform(name + ".start");
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name + ".density", this.density);
        shaderProgram.setUniform(name + ".start", this.start);
    }
}
package net.comsoria.engine.view;

import net.comsoria.engine.Scene;
import net.comsoria.engine.utils.Timer;
import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.engine.view.GLSL.programs.CustomShaderProgram;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.graph.*;
import net.comsoria.engine.view.graph.mesh.Mesh;
import org.joml.Matrix4f;

import java.io.Closeable;
import java.io.IOException;
import java.util.Arrays;

import static org.lwjgl.opengl.EXTFramebufferObject.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL14.GL_DEPTH_COMPONENT32;
import static org.lwjgl.opengl.GL20.glDrawBuffers;
import static org.lwjgl.opengl.GL30.*;
import static org.lwjgl.opengl.GL30.GL_FRAMEBUFFER;
import static org.lwjgl.system.MemoryUtil.NULL;

public class FrameBuffer implements Renderable {
    private final Mesh mesh;

    private int fbo, drb;

    public FrameBuffer(int width, int height, String vertex, String fragment) throws IOException {
        this(width, height, FrameBuffer.generateFrameBufferShader(vertex, fragment));
    }

    public FrameBuffer(int width, int height, ShaderProgram shaderProgram) throws IOException {
        BufferAttribute vertices = new BufferAttribute(new float[] {
                -1, -1,
                1, -1,
                1, 1,
                -1, 1
        }, 2);
        Geometry geometry = new Geometry(new Tuple<>(Arrays.asList(vertices), new int[] {
                0, 1, 2, 0, 2, 3
        }));

        Material material = new Material();
        mesh = new Mesh(geometry, material, shaderProgram);
        mesh.initShaderProgram();

        setSize(width, height);
    }

    public void setSize(int width, int height) {
        cleanup();
        mesh.material.textures.add(new Texture(width, height));
        mesh.material.textures.add(new Texture(() -> glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL)));

        fbo = glGenFramebuffersEXT();
        this.bind();

        drb = glGenRenderbuffers();
        glBindRenderbuffer(GL_RENDERBUFFER, drb);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, drb);

        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, mesh.material.textures.get(0).getId(), 0);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, mesh.material.textures.get(1).getId(), 0);

        glDrawBuffers(new int[] {GL_COLOR_ATTACHMENT0, GL_DEPTH_ATTACHMENT});

        if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE_EXT) {
            System.out.println("ERROR");
            return;
        }

        FrameBuffer.unbind();
    }

    public void bind() {
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    }

    public Mesh getMesh() {
        return mesh;
    }

    public int getID() {
        return fbo;
    }

    @Override
    public Closeable render(Transformation transformation, Scene scene, RenderData renderData, Window window) throws Exception {
        return this.mesh.render(transformation, scene, renderData, window);
    }

    public void cleanup() {
        glDeleteFramebuffersEXT(fbo);
        if (this.mesh.material.isTextured()) {
            int len = this.mesh.material.textures.size();
            for (int i = 0; i < len; i++) {
                this.mesh.material.textures.get(0).cleanup();
                this.mesh.material.textures.remove(0);
            }
        }
        glDeleteRenderbuffers(drb);
    }

    @Override
    public boolean shouldRender() {
        return true;
    }

    @Override
    public RenderOrder getRenderOrder() {
        return RenderOrder.Any;
    }

    public static void unbind() {
        glBindFramebufferEXT(GL_FRAMEBUFFER, 0);
    }

    public static ShaderProgram generateFrameBufferShader(String vertex, String fragment) {
        return new CustomShaderProgram(vertex, fragment, Arrays.asList("time"), Arrays.asList("colorTexture", "depthTexture")) {
            @Override
            public void setupScene(Scene scene, Transformation transformation) {
                this.setUniform("time", Timer.getTime());
            }

            @Override
            public void setupMesh(Mesh mesh1, Matrix4f modelMatrix, Transformation transformation) {

            }
        };
    }
}
package net.comsoria.engine.view;

import net.comsoria.engine.math.Rectangle;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.batch.RenderData;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.opengl.GL11.*;

public class FrameBufferRenderer {
    public List<FrameBuffer> frameBuffers = new ArrayList<>();
    private Transformation transformation = new Transformation();

    public void renderBase(Window window) {
        if (window.isResized()) {
            glViewport(0, 0, window.getWidth(), window.getHeight());
            for (FrameBuffer frameBuffer : frameBuffers) frameBuffer.setSize(window.getWidth(), window.getHeight());
            window.setResized(false);
        }
    }

    public void clear() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    public void bindInitialFramebuffer() {
        if (frameBuffers.size() != 0) {
            frameBuffers.get(0).bind();
        }
        clear();
    }

    public void renderFramebuffers(Window window) throws Exception {
        List<Closeable> closeables = new ArrayList<>();

        for (int i = 1; i < frameBuffers.size(); i++) {
            FrameBuffer frameBuffer = frameBuffers.get(i);
            frameBuffer.bind();

            clear();
            closeables.add(frameBuffers.get(i - 1).render(transformation, null, RenderData.defaultRenderData, window));
        }

        if (frameBuffers.size() != 0) {
            FrameBuffer.unbind();

            clear();
            closeables.add(frameBuffers.get(frameBuffers.size() - 1).render(transformation, null, RenderData.defaultRenderData, window));
        }

        for (Closeable closeable : closeables) closeable.close();
    }
}
package net.comsoria.engine.view.GLSL;

public class GLSLException extends RuntimeException {
    public GLSLException(String message) {
        super(message);
    }
}
package net.comsoria.engine.view.GLSL;

public interface GLSLUniformBindable {
    void set(ShaderProgram shaderProgram, String name);
}
package net.comsoria.engine.view.GLSL.matrices;

import net.comsoria.engine.view.Camera;
import net.comsoria.engine.view.Window;
import org.joml.Matrix4f;
import org.joml.Vector3f;

public final class Transformation {
    private Matrix4f projection = new Matrix4f();
    private Matrix4f ortho = new Matrix4f();

    private Matrix4f view = new Matrix4f();
    private Matrix4f viewNoTranslation = new Matrix4f();
    private Matrix4f viewNoRotation = new Matrix4f();

    private Matrix4f viewData = new Matrix4f();

    public Matrix4f getProjectionMatrix(Window window, Camera camera) {
        float aspectRatio = window.getWidth() / window.getHeight();

        projection = new Matrix4f();
        projection.perspective(camera.fov, aspectRatio, camera.near, camera.far);

        return projection;
    }

    public Matrix4f getProjection() {
        return projection;
    }

    private Matrix4f getViewMatrix(Camera camera, boolean rot, boolean trans) {
        Vector3f cameraPos = camera.position;
        Vector3f rotation = camera.rotation;

        viewData.identity();
        if (rot) viewData.rotate((float) Math.toRadians(rotation.x), new Vector3f(1, 0, 0))
                                .rotate((float) Math.toRadians(rotation.y), new Vector3f(0, 1, 0));

        if (trans) viewData.translate(-cameraPos.x, -cameraPos.y, -cameraPos.z);

        return new Matrix4f(viewData);
    }

    public Matrix4f getView(Camera camera) {
        view = getViewMatrix(camera, true, true);
        return view;
    }

    public Matrix4f getView() {
        return view;
    }

    public Matrix4f getViewNoRotation(Camera camera) {
        viewNoRotation = getViewMatrix(camera, false, true);
        return viewNoRotation;
    }

    public Matrix4f getViewNoRotation() {
        return viewNoRotation;
    }

    public Matrix4f getViewNoTranslation(Camera camera) {
        viewNoTranslation = getViewMatrix(camera, true, false);
        return viewNoTranslation;
    }

    public Matrix4f getViewNoTranslation() {
        return viewNoTranslation;
    }

    public Matrix4f getOrthoProjectionMatrix(float left, float right, float bottom, float top) {
        ortho.identity();
        ortho.setOrtho2D(left, right, bottom, top);

        return ortho;
    }

    public Matrix4f getOrtho() {
        return ortho;
    }

    public static Matrix4f updateGenericViewMatrix(Vector3f position, Vector3f rotation, Matrix4f matrix) {
        // First do the rotation so camera rotates over its position
        return matrix.rotationX((float)Math.toRadians(rotation.x))
                .rotateY((float)Math.toRadians(rotation.y))
                .translate(-position.x, -position.y, -position.z);
    }
}
package net.comsoria.engine.view.GLSL.programs;

import net.comsoria.engine.view.GLSL.ShaderProgram;

import java.util.List;

public abstract class CustomShaderProgram extends ShaderProgram {
    private final String vertex;
    private final String fragment;
    private final List<String> uniforms;

    public CustomShaderProgram(String vertex, String fragment, List<String> uniforms, List<String> textures) {
        super();

        this.vertex = vertex;
        this.fragment = fragment;
        this.uniforms = uniforms;
        this.textures.addAll(textures);
    }

    public CustomShaderProgram(String vertex, String fragment, List<String> uniforms) {
        super();

        this.vertex = vertex;
        this.fragment = fragment;
        this.uniforms = uniforms;
    }

    @Override
    public void init() {
        this.create(vertex, fragment);
        for (String name : uniforms) this.createUniform(name);
        for (String name : textures) this.createUniform(name);
    }
}
package net.comsoria.engine.view.GLSL.programs;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.FadeFog;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.light.DirectionalLight;
import net.comsoria.engine.view.graph.mesh.Mesh;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;

import java.io.IOException;

public class FadeOutShaderProgram extends ShaderProgram3D {
    private final String vertex;
    private final String fragment;

    public FadeOutShaderProgram(String vertex, String fragment) {
        super();

        this.vertex = vertex;
        this.fragment = fragment;
    }

    public void init() throws IOException {
        this.create(vertex, fragment);

        this.createUniform("projectionMatrix");
        this.createUniform("modelViewMatrix");
        this.createUniform("modelViewMatrixNoRot");

        this.createUniform("ambientLight");

        FadeFog.create(this, "fog");
        DirectionalLight.create(this, "directionalLight");

        this.createUniform("color1");
        this.createUniform("color2");
        this.createUniform("sunDirection");
    }

    @Override
    public void setupScene(Scene scene, Transformation transformation) {
        this.setUniform("projectionMatrix", transformation.getProjection());
        this.setUniform("ambientLight", scene.light.ambientLight.getVec3());

        this.setUniform("fog", scene.fog);

        DirectionalLight currDirLight = new DirectionalLight(scene.light.directionalLight);
        Vector4f dir = new Vector4f(currDirLight.direction, 0);
        dir.mul(transformation.getView());
        currDirLight.direction = new Vector3f(dir.x, dir.y, dir.z);
        this.setUniform("directionalLight", currDirLight);

        this.setUniform("color1", scene.sky.getMainColor().getVec3());
        this.setUniform("color2", scene.sky.getSecondColor().getVec3());
        this.setUniform("sunDirection", scene.light.directionalLight.direction);
    }

    @Override
    public void setupMesh(Mesh mesh, Matrix4f modelMatrix, Transformation transformation) {
        this.setUniform("modelViewMatrix", new Matrix4f(transformation.getView()).mul(modelMatrix));
        this.setUniform("modelViewMatrixNoRot", new Matrix4f(transformation.getViewNoRotation()).mul(modelMatrix));
    }
}
package net.comsoria.engine.view.GLSL.programs;

import net.comsoria.engine.Scene;
import net.comsoria.engine.loaders.FileLoader;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.graph.mesh.Mesh;
import org.joml.Matrix4f;

import java.io.IOException;

public class ShaderProgram2D extends ShaderProgram {
    public ShaderProgram2D() {
        super();
    }

    public void init() throws IOException {
        this.create(FileLoader.loadResourceAsStringFromPath("$shaders/hud/hud_vertex.v.glsl"),
                FileLoader.loadResourceAsStringFromPath("$shaders/hud/hud_fragment.f.glsl"));

        this.createUniform("projModelMatrix");
        this.createUniform("color");
        this.createUniform("hasTexture");
    }

    @Override
    public void setupScene(Scene scene, Transformation transformation) {

    }

    @Override
    public void setupMesh(Mesh mesh, Matrix4f modelViewMatrix, Transformation transformation) {
        this.setUniform("projModelMatrix", modelViewMatrix);
        this.setUniform("color", mesh.material.ambientColour);
        this.setUniform("hasTexture", mesh.material.textures.size() == 0? 0:1);

//        System.out.println(mesh.material.ambientColour);
//        System.out.println(mesh.geometry.getVertexCount());
//        System.out.println("-------");
    }
}
package net.comsoria.engine.view.GLSL.programs;

import net.comsoria.engine.Scene;
import net.comsoria.engine.loaders.FileLoader;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.light.DirectionalLight;
import net.comsoria.engine.view.light.PointLight;
import net.comsoria.engine.view.light.SpotLight;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.mesh.Mesh;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;

import java.io.IOException;
import java.util.List;

public class ShaderProgram3D extends ShaderProgram {
    public ShaderProgram3D() {
        super();
    }

    public void init() throws IOException {
        this.create(FileLoader.loadResourceAsStringFromPath("$shaders/main/vertex.v.glsl"),
                FileLoader.loadResourceAsStringFromPath("$shaders/main/fragment.f.glsl"));

        this.createUniform("projectionMatrix");
        this.createUniform("modelViewMatrix");

        Material.create(this, "material");

        this.createUniform("specularPower");
        this.createUniform("ambientLight");

        for (int i = 0; i < 5; i++) {
            PointLight.create(this, "pointLights[" + i + "]");
        }

        for (int i = 0; i < 5; i++) {
            SpotLight.create(this, "spotLights[" + i + "]");
        }

        DirectionalLight.create(this, "directionalLight");
    }


    @Override
    public void setupScene(Scene scene, Transformation transformation) {
        this.setUniform("projectionMatrix", transformation.getProjection());

        this.setUniform("ambientLight", scene.light.ambientLight);
        this.setUniform("specularPower", 10f);

        List<PointLight> pointLightList = scene.light.pointLightList;
        int numLights = pointLightList != null ? pointLightList.size() : 0;
        for (int i = 0; i < numLights; i++) {
            PointLight currPointLight = new PointLight(pointLightList.get(i));

            Vector3f lightPos = currPointLight.position;
            Vector4f aux = new Vector4f(lightPos, 1);
            aux.mul(transformation.getView());

            lightPos.x = aux.x;
            lightPos.y = aux.y;
            lightPos.z = aux.z;

            currPointLight.set(this, "pointLights[" + i + "]");
        }

        List<SpotLight> spotLightList = scene.light.spotLightList;
        numLights = spotLightList != null ? spotLightList.size() : 0;

        for (int i = 0; i < numLights; i++) {
            SpotLight currSpotLight = new SpotLight(spotLightList.get(i));

            Vector4f dir = new Vector4f(currSpotLight.coneDirection, 0);
            dir.mul(transformation.getView());
            currSpotLight.coneDirection = new Vector3f(dir.x, dir.y, dir.z);

            Vector3f lightPos = currSpotLight.pointLight.position;
            Vector4f aux = new Vector4f(lightPos, 1);
            aux.mul(transformation.getView());
            lightPos.x = aux.x;
            lightPos.y = aux.y;
            lightPos.z = aux.z;

            currSpotLight.set(this, "spotLights[" + i + "]");
        }

        DirectionalLight currDirLight = new DirectionalLight(scene.light.directionalLight);
        Vector4f dir = new Vector4f(currDirLight.direction, 0);
        dir.mul(transformation.getView());
        currDirLight.direction = new Vector3f(dir.x, dir.y, dir.z);
        this.setUniform("directionalLight", currDirLight);
    }

    @Override
    public void setupMesh(Mesh mesh, Matrix4f modelViewMatrix, Transformation transformation) {
        this.setUniform("modelViewMatrix", modelViewMatrix);
        this.setUniform("material", mesh.material);
    }
}
package net.comsoria.engine.view.GLSL;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.graph.mesh.Mesh;
import org.joml.Matrix4f;
import org.joml.Vector2f;
import org.joml.Vector3f;
import org.joml.Vector4f;
import org.lwjgl.system.MemoryStack;

import java.io.Closeable;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.*;

import static org.lwjgl.opengl.GL20.*;

public abstract class ShaderProgram implements Closeable {
    private final int programId;
    private int vertexShaderId;
    private int fragmentShaderId;
    private final Map<String, Integer> uniforms;
    public final List<String> textures;
    private boolean updated = false;

    public ShaderProgram() {
        this(glCreateProgram());

    }

    public ShaderProgram(int id) {
        programId = id;
        if (programId == 0) {
            throw new GLSLException("Failed to create Shader program");
        }
        uniforms = new HashMap<>();
        textures = new ArrayList<>();
    }

    public void createUniform(String uniformName) {
        int uniformLocation = glGetUniformLocation(programId, uniformName);
        if (uniformLocation < 0) {
            throw new GLSLException("Uniform '" + uniformName + "' not used in GLSL. Failed to create.");
        }
        uniforms.put(uniformName, uniformLocation);
    }

    public void createTextureUniform(String name) {
        textures.add(name);
        createUniform(name);
    }

    public void createListUniform(String name, int length) {
        for (int i = 0; i < length; i++) {
            createUniform(name + "[" + i + "]");
        }
    }

    public void setUniform(String uniformName, Matrix4f value) {
        try (MemoryStack stack = MemoryStack.stackPush()) {
            // Dump the matrix into a float buffer
            FloatBuffer fb = stack.mallocFloat(16);
            value.get(fb);
            try {
                glUniformMatrix4fv(uniforms.get(uniformName), false, fb);
            } catch (Exception e) {
                throw new GLSLException("Failed to find uniform of name '" + uniformName + "'");
            }

        }
    }

    public void setUniform(String uniformName, int value) {
        try {
            glUniform1i(uniforms.get(uniformName), value);
        } catch (Exception e) {
            throw new GLSLException("Failed to find uniform of name '" + uniformName + "'");
        }
    }

    public void setUniform(String uniformName, float value) {
        try {
            glUniform1f(uniforms.get(uniformName), value);
        } catch (Exception e) {
            throw new GLSLException("Failed to find uniform of name '" + uniformName + "'");
        }
    }

    public void setUniform(String uniformName, Vector2f value) {
        try {
            glUniform2f(uniforms.get(uniformName), value.x, value.y);
        } catch (Exception e) {
            throw new GLSLException("Failed to find uniform of name '" + uniformName + "'");
        }
    }

    public void setUniform(String uniformName, Vector3f value) {
        try {
            glUniform3f(uniforms.get(uniformName), value.x, value.y, value.z);
        } catch (Exception e) {
            throw new GLSLException("Failed to find uniform of name '" + uniformName + "'");
        }
    }

    public void setUniform(String uniformName, Vector4f value) {
        try {
            glUniform4f(uniforms.get(uniformName), value.x, value.y, value.z, value.w);
        } catch (Exception e) {
            throw new GLSLException("Failed to find uniform of name '" + uniformName + "'");
        }
    }

    public void setUniform(String name, GLSLUniformBindable object) {
        object.set(this, name);
    }


    private void createVertexShader(String shaderCode) {
        vertexShaderId = createShader(shaderCode, GL_VERTEX_SHADER);
    }

    private void createFragmentShader(String shaderCode) {
        fragmentShaderId = createShader(shaderCode, GL_FRAGMENT_SHADER);
    }

    private int createShader(String shaderCode, int shaderType) {
        int shaderId = glCreateShader(shaderType);
        if (shaderId == 0) {
            throw new GLSLException("Error creating shader. Type: " + shaderType);
        }

        glShaderSource(shaderId, shaderCode);
        glCompileShader(shaderId);

        if (glGetShaderi(shaderId, GL_COMPILE_STATUS) == 0) {
            throw new GLSLException("Error compiling Shader code: " + glGetShaderInfoLog(shaderId, 1024));
        }

        glAttachShader(programId, shaderId);

        return shaderId;
    }


    public void open() {
        this.updated = true;
}

    public void close() {
        this.updated = false;
    }

    public boolean isUpdated() {
        return this.updated;
    }


    public abstract void setupScene(Scene scene, Transformation transformation);

    public abstract void setupMesh(Mesh mesh, Matrix4f modelMatrix, Transformation transformation);


    private void link() {
        glLinkProgram(programId);
        if (glGetProgrami(programId, GL_LINK_STATUS) == 0) {
            throw new GLSLException("Error linking Shader code: " + glGetProgramInfoLog(programId, 1024));
        }

        if (vertexShaderId != 0) {
            glDetachShader(programId, vertexShaderId);
        }
        if (fragmentShaderId != 0) {
            glDetachShader(programId, fragmentShaderId);
        }

        glValidateProgram(programId);
        if (glGetProgrami(programId, GL_VALIDATE_STATUS) == 0) {
            System.err.println("Warning validating Shader code: " + glGetProgramInfoLog(programId, 1024));
        }
    }

    public void bind() {
        glUseProgram(programId);
    }

    public void unbind() {
        glUseProgram(0);
    }

    public void cleanup() {
        unbind();
        if (programId != 0) {
            glDeleteProgram(programId);
        }
    }

    public void create(String vertex, String fragment) {
        this.createVertexShader(vertex);
        this.createFragmentShader(fragment);
        this.link();
    }

    public void init() throws IOException {

    }


    public int getProgramId() {
        return programId;
    }
}
package net.comsoria.engine.view.graph;

import org.joml.Vector2f;
import org.joml.Vector3f;
import org.joml.Vector3i;
import org.lwjgl.system.MemoryUtil;

import java.nio.FloatBuffer;
import java.util.List;

import static org.lwjgl.opengl.GL11.GL_FLOAT;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL15.GL_ARRAY_BUFFER;
import static org.lwjgl.opengl.GL15.GL_STATIC_DRAW;
import static org.lwjgl.opengl.GL20.glDisableVertexAttribArray;
import static org.lwjgl.opengl.GL20.glEnableVertexAttribArray;
import static org.lwjgl.opengl.GL20.glVertexAttribPointer;

public class BufferAttribute {
    private float[] data;
    private final int indices;

    private int id;

    public static BufferAttribute create3f(List<Vector3f> vecs) {
        float[] data = new float[vecs.size() * 3];
        for (int i = 0; i < vecs.size(); i++) {
            Vector3f vec = vecs.get(i);

            data[(i * 3)] = vec.x;
            data[(i * 3) + 1] = vec.y;
            data[(i * 3) + 2] = vec.z;
        }

        return new BufferAttribute(data, 3);
    }

    public static BufferAttribute create2f(List<Vector2f> vecs) {
        float[] data = new float[vecs.size() * 2];
        for (int i = 0; i < vecs.size(); i++) {
            Vector2f vec = vecs.get(i);

            data[(i * 2)] = vec.x;
            data[(i * 2) + 1] = vec.y;
        }

        return new BufferAttribute(data, 2);
    }

    public BufferAttribute(float[] data, int indices) {
        this.data = data;
        this.indices = indices;
    }

    public void bind(int index) {
        FloatBuffer buffer = null;
        try {
            id = glGenBuffers();
            buffer = MemoryUtil.memAllocFloat(data.length);
            buffer.put(data).flip();
            glBindBuffer(GL_ARRAY_BUFFER, id);
            glBufferData(GL_ARRAY_BUFFER, buffer, GL_STATIC_DRAW);
            glVertexAttribPointer(index, indices, GL_FLOAT, false, 0, 0);
        } finally {
            if (buffer != null) MemoryUtil.memFree(buffer);
        }
    }

    public void cleanup() {
        glDeleteBuffers(this.id);
    }

    void enable(int index) {
        glEnableVertexAttribArray(index);
    }

    void disable(int index) {
        glDisableVertexAttribArray(index);
    }

    public float[] get() {
        return data;
    }

    public float get(int x) {
        return data[x];
    }

    public void set(int x, float value) {
        data[x] = value;
    }

    public void add(int x, float value) {
        data[x] += value;
    }

    public void set(float[] value) {
        data = value;
    }

    public void update(int start, int length) {
        float[] newData = new float[length];
        System.arraycopy(data, start, newData, 0, length);

        glBindBuffer(GL_ARRAY_BUFFER, this.id);
        glBufferSubData(GL_ARRAY_BUFFER, start, newData);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    public Vector3f getVec3(int index) {
        return new Vector3f(data[index * 3], data[(index * 3) + 1], data[(index * 3) + 2]);
    }

    public void updateAll() {
        this.update(0, this.size());
    }

    public int size() {
        return this.data.length;
    }

    public int parts() {
        return this.data.length / this.indices;
    }

    public void translate(float... dimensions) {
        for (int i = 0; i < data.length; i++) {
            data[i] += dimensions[i % dimensions.length];
        }
    }

    public BufferAttribute clone() {
        return new BufferAttribute(this.data.clone(), this.indices);
    }
}
package net.comsoria.engine.view.graph;

import net.comsoria.engine.utils.Tuple;
import org.lwjgl.system.MemoryUtil;

import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.*;
import static org.lwjgl.opengl.GL30.glBindVertexArray;
import static org.lwjgl.opengl.GL30.glDeleteVertexArrays;
import static org.lwjgl.opengl.GL30.glGenVertexArrays;

public class Geometry {
    private final int vaoId;
    private final int vertexCount;
    private final int indicesBufferID;
    private int culledFace = -1;

    private final List<BufferAttribute> vbos;

    public Geometry(Tuple<List<BufferAttribute>, int[]> data) {
        this(data.getA(), data.getB());
    }

    public Geometry(List<BufferAttribute> attributes, int[] indices) {
        vertexCount = indices.length;
        this.vbos = attributes;

        vaoId = glGenVertexArrays();
        glBindVertexArray(vaoId);

        for (int i = 0; i < attributes.size(); i++) attributes.get(i).bind(i);

        IntBuffer indicesBuffer = null;
        try {
            indicesBufferID = glGenBuffers();
            indicesBuffer = MemoryUtil.memAllocInt(indices.length);
            indicesBuffer.put(indices).flip();
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesBufferID);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer, GL_STATIC_DRAW);
        } finally {
            if (indicesBuffer != null) {
                MemoryUtil.memFree(indicesBuffer);
            }
        }

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    public void setCullFace(int face) {
        this.culledFace = face;
    }

    public void enableCull() {
        if (this.culledFace != -1) {
            glEnable(GL_CULL_FACE);
            glCullFace(this.culledFace);
        }
    }

    public void disableCull() {
        if (this.culledFace != -1) {
            glDisable(GL_CULL_FACE);
        }
    }

    public int getVaoId() {
        return vaoId;
    }

    public int getVertexCount() {
        return vertexCount;
    }

    public void cleanup() {
        glDisableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        for (BufferAttribute vbo : vbos) vbo.cleanup();
        glDeleteBuffers(indicesBufferID);

        glBindVertexArray(0);
        glDeleteVertexArrays(vaoId);
    }

    public void bindAttributes() {
        for (int i = 0; i < vbos.size(); i++) {
            vbos.get(i).enable(i);
        }
    }

    public void unbindAttributes() {
        for (int i = 0; i < vbos.size(); i++) {
            vbos.get(i).disable(i);
        }
    }

    public BufferAttribute getVBO(int index) {
        return this.vbos.get(index);
    }

    public void updateVBO(int index) {
        this.bind();
        this.vbos.get(index).updateAll();
        this.unbind();
    }

    public void bind() {
        glBindVertexArray(vaoId);
    }

    public void unbind() {
        glBindVertexArray(0);
    }
}
package net.comsoria.engine.view.graph;

import net.comsoria.engine.view.color.Color4;
import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;

import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.opengl.GL11.glDeleteTextures;

public class Material implements GLSLUniformBindable {
    private static final Color4 DEFAULT_COLOUR = Color4.WHITE;
    public Color4 ambientColour;
    public Color4 diffuseColour;
    public Color4 specularColour;
    public float reflectance;
    public List<Texture> textures;

    public Material() {
        this.ambientColour = DEFAULT_COLOUR.clone();
        this.diffuseColour = DEFAULT_COLOUR.clone();
        this.specularColour = DEFAULT_COLOUR.clone();
        this.textures = new ArrayList<>();
        this.reflectance = 0;
    }

    public Material(Color4 colour, float reflectance) {
        this(colour, colour, colour, null, reflectance);
    }

    public Material(Texture texture) {
        this(DEFAULT_COLOUR.clone(), DEFAULT_COLOUR.clone(), DEFAULT_COLOUR.clone(), texture, 0);
    }

    public Material(Texture texture, float reflectance) {
        this(DEFAULT_COLOUR.clone(), DEFAULT_COLOUR.clone(), DEFAULT_COLOUR.clone(), texture, reflectance);
    }

    public Material(Color4 ambientColour, Color4 diffuseColour, Color4 specularColour, Texture texture, float reflectance) {
        this.ambientColour = ambientColour;
        this.diffuseColour = diffuseColour;
        this.specularColour = specularColour;

        this.textures = new ArrayList<>();
        if (texture != null) this.textures.add(texture);

        this.reflectance = reflectance;
    }

    public void cleanup() {
        for (Texture texture : textures) texture.cleanup();
    }

    public boolean isTextured() {
        return this.textures.size() != 0 && this.textures.get(0) != null;
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name + ".ambient", this.ambientColour);
        shaderProgram.setUniform(name + ".diffuse", this.diffuseColour);
        shaderProgram.setUniform(name + ".specular", this.specularColour);
        shaderProgram.setUniform(name + ".reflectance", this.reflectance);
    }

    public static void create(ShaderProgram shaderProgram, String name) {
        shaderProgram.createUniform(name + ".ambient");
        shaderProgram.createUniform(name + ".diffuse");
        shaderProgram.createUniform(name + ".specular");
        shaderProgram.createUniform(name + ".hasTexture");
        shaderProgram.createUniform(name + ".reflectance");
    }
}
package net.comsoria.engine.view.graph.mesh;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.Renderable;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.Texture;
import org.joml.Matrix4f;
import org.joml.Vector3f;

import java.io.Closeable;
import java.io.IOException;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL13.GL_TEXTURE0;
import static org.lwjgl.opengl.GL13.glActiveTexture;

public class Mesh implements Renderable {
    public Material material;
    public Geometry geometry;

    public final Vector3f position = new Vector3f();
    public final Vector3f rotation = new Vector3f();
    public float scale = 1f;
    public boolean visible = true;
    public RenderOrder renderPosition = RenderOrder.Any;

    public ShaderProgram shaderProgram = null;

    protected final static Matrix4f modelViewMatrix = new Matrix4f();

    public Mesh(Geometry geometry, Material material, ShaderProgram shaderProgram) {
        this.geometry = geometry;
        this.material = material;
        this.shaderProgram = shaderProgram;
    }

    public void initShaderProgram() throws IOException {
        this.geometry.bind();
        this.shaderProgram.init();
        this.geometry.unbind();
    }

    public void cleanup() {
        geometry.cleanup();
        material.cleanup();
        shaderProgram.cleanup();
    }

    @Override public boolean shouldRender() {
        return visible;
    }

    @Override public RenderOrder getRenderOrder() {
        return renderPosition;
    }

    public Matrix4f getModelViewMatrix(Transformation transformation) {
        modelViewMatrix.identity().translate(this.position).
                rotateX((float) Math.toRadians(-rotation.x)).
                rotateY((float) Math.toRadians(-rotation.y)).
                rotateZ((float) Math.toRadians(-rotation.z)).
                scale(this.scale);

        Matrix4f viewCurr = new Matrix4f(transformation.getView());
        return viewCurr.mul(modelViewMatrix);
    }

    @Override public Closeable render(Transformation transformation, Scene scene, RenderData renderData, Window window) throws Exception {
        if (renderData.shouldBindOwnGeometry()) this.geometry.bind();
        if (renderData.shouldBindOwnShaderProgram()) this.shaderProgram.bind();

        if (!this.shaderProgram.isUpdated()) {
            this.shaderProgram.open();
            this.shaderProgram.setupScene(scene, transformation);
        }

        this.shaderProgram.setupMesh(this, getModelViewMatrix(transformation), transformation);

        if (this.material.textures.size() != this.shaderProgram.textures.size())
            throw new Exception("Unequal textures to texture uniforms");

        for (int i = 0; i < this.material.textures.size(); i++) {
            Texture texture = this.material.textures.get(i);
            this.shaderProgram.setUniform(this.shaderProgram.textures.get(i), i);

            glActiveTexture(GL_TEXTURE0 + i);
            texture.bind();
        }

        this.geometry.enableCull();
        this.geometry.bindAttributes();
        glDrawElements(GL_TRIANGLES, this.geometry.getVertexCount(), GL_UNSIGNED_INT, 0);
        this.geometry.unbindAttributes();
        this.geometry.disableCull();

        if (this.material.textures.size() != 0) Texture.unbind();

        if (renderData.shouldBindOwnGeometry()) this.geometry.unbind();
        if (renderData.shouldBindOwnShaderProgram()) this.shaderProgram.unbind();

        return this.shaderProgram;
    }

    public Mesh clone() {
        Mesh mesh = new Mesh(this.geometry, this.material, this.shaderProgram);
        mesh.position.set(this.position);
        mesh.rotation.set(this.rotation);
        mesh.scale = this.scale;
        mesh.visible = this.visible;
        mesh.renderPosition = this.renderPosition;
        return mesh;
    }
}
package net.comsoria.engine.view.graph.mesh;

import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import org.joml.Matrix4f;

public class Mesh2D extends Mesh {
    public Mesh2D(Geometry geometry, Material material, ShaderProgram shaderProgram) {
        super(geometry, material, shaderProgram);
    }

    @Override
    public Matrix4f getModelViewMatrix(Transformation transformation) {
        Matrix4f modelMatrix = new Matrix4f();
        modelMatrix.identity().translate(this.position).
                rotateX((float) Math.toRadians(-rotation.x)).
                rotateY((float) Math.toRadians(-rotation.y)).
                rotateZ((float) Math.toRadians(-rotation.z)).
                scale(this.scale);

        Matrix4f orthoMatrixCurr = new Matrix4f(transformation.getOrtho());
        orthoMatrixCurr.mul(modelMatrix);
        return orthoMatrixCurr;
    }

    public Mesh2D clone() {
        return (Mesh2D) super.clone();
    }
}
package net.comsoria.engine.view.graph.mesh;

import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import org.joml.Matrix4f;

public class NoViewMatrixMesh extends Mesh {
    public NoViewMatrixMesh(Geometry geometry, Material material, ShaderProgram shaderProgram) {
        super(geometry, material, shaderProgram);
    }

    @Override
    public Matrix4f getModelViewMatrix(Transformation transformation) {
        modelViewMatrix.identity().translate(this.position).
                rotateX((float) Math.toRadians(-rotation.x)).
                rotateY((float) Math.toRadians(-rotation.y)).
                rotateZ((float) Math.toRadians(-rotation.z)).
                scale(this.scale);

        return modelViewMatrix;

        //Note that what is return is ->NOT<- a model view matrix but is just a model matrix instead
    }

    public NoViewMatrixMesh clone() {
        return (NoViewMatrixMesh) super.clone();
    }
}
package net.comsoria.engine.view.graph.mesh;

import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import org.joml.Matrix4f;

import static org.lwjgl.opengl.GL11.GL_FRONT;

public class SkyBox extends Mesh {
    public SkyBox(Geometry geometry, Material material, ShaderProgram shaderProgram) {
        super(geometry, material, shaderProgram);
        this.renderPosition = RenderOrder.End;
        this.geometry.setCullFace(GL_FRONT);
    }

    @Override
    public Matrix4f getModelViewMatrix(Transformation transformation) {
        modelViewMatrix.identity().translate(this.position).
                rotateX((float) Math.toRadians(-rotation.x)).
                rotateY((float) Math.toRadians(-rotation.y)).
                rotateZ((float) Math.toRadians(-rotation.z)).
                scale(this.scale);

        Matrix4f viewCurr = new Matrix4f(transformation.getViewNoTranslation());
        return viewCurr.mul(modelViewMatrix);
    }

    public SkyBox clone() {
        return (SkyBox) super.clone();
    }
}
package net.comsoria.engine.view.graph;

import de.matthiasmann.twl.utils.PNGDecoder;
import net.comsoria.engine.loaders.FileLoader;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.nio.ByteBuffer;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL11.glDeleteTextures;
import static org.lwjgl.opengl.GL12.GL_CLAMP_TO_EDGE;
import static org.lwjgl.opengl.GL14.GL_DEPTH_COMPONENT24;
import static org.lwjgl.opengl.GL30.glGenerateMipmap;

public class Texture {
    private final int id;

    public Texture(String fileName) throws Exception {
        this(loadTexture(fileName));
    }

    public Texture(InputStream is) throws Exception {
        this(loadTexture(is));
    }

    public Texture(int id) {
        this.id = id;
    }

    public Texture(int width, int height) {
        this(generateTexture(width, height));
    }

    public Texture(Runnable textureUploader) {
        this(generateTexture(textureUploader));
    }

    public void bind() {
        glBindTexture(GL_TEXTURE_2D, id);
    }

    public int getId() {
        return id;
    }

    public static int loadTexture(String fileName) throws Exception {
        return loadTexture(FileLoader.loadResourceAsStream(fileName));
    }

    private static int loadTexture(InputStream inputStream) throws Exception {
        // Load Texture file
        PNGDecoder decoder = new PNGDecoder(inputStream);

        // Load texture contents into a byte buffer
        ByteBuffer buf = ByteBuffer.allocateDirect(
                4 * decoder.getWidth() * decoder.getHeight());
        decoder.decode(buf, decoder.getWidth() * 4, PNGDecoder.Format.RGBA);
        buf.flip();

        // Create a new OpenGL texture
        return generateTexture(buf, decoder.getWidth(), decoder.getHeight());
    }

    private static int generateTexture(ByteBuffer buffer, int width, int height) {
        return generateTexture(() -> glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer));
    }

    private static int generateTexture(int width, int height) {
        return generateTexture(() -> glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0));
    }

    private static int generateTexture(Runnable textureUploader) {
        int textureId = glGenTextures();
        // Bind the texture
        glBindTexture(GL_TEXTURE_2D, textureId);

        // Tell OpenGL how to unpack the RGBA bytes. Each component is 1 byte size
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // Upload the texture data
        textureUploader.run();
        // Generate Mip Map
        glGenerateMipmap(GL_TEXTURE_2D);
        return textureId;
    }

    public void cleanup() {
        glDeleteTextures(id);
    }

    public static void unbind() {
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}
package net.comsoria.engine.view.input;

import net.comsoria.engine.view.Window;
import org.lwjgl.glfw.GLFWKeyCallback;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.lwjgl.glfw.GLFW.GLFW_RELEASE;
import static org.lwjgl.glfw.GLFW.glfwSetKeyCallback;

public class KeyInput extends GLFWKeyCallback {
    private Map<Integer, Key> keys = new HashMap<>();
    private List<KeyListener> keyListeners = new ArrayList<>();

    public void init(Window window) {
        glfwSetKeyCallback(window.getWindowHandle(), this);
    }

    public void addListener(KeyListener listener) {
        keyListeners.add(listener);
    }

    @Override
    public void invoke(long window, int key, int scancode, int action, int mods) {
        for (KeyListener listener : keyListeners) {
            if (listener.listenedKey(key)) {
                listener.keyEvent.call(key, action);
                if (!listener.passive) break;
            }
        }

        Key objKey;

        if (!keys.keySet().contains(key)) {
            objKey = new Key();
            keys.put(key, objKey);
        } else objKey = keys.get(key);

        objKey.action = action;
    }

    private boolean keyExists(int keyCode) {
        return keys.keySet().contains(keyCode);
    }

    public boolean isKeyPressed(int keyCode) {
        return keyExists(keyCode) && keys.get(keyCode).isPressed();
    }
}

class Key {
    public int action = GLFW_RELEASE;

    public boolean isPressed() {
        return action != GLFW_RELEASE;
    }
}package net.comsoria.engine.view.input;

import net.comsoria.engine.utils.Utils;

import java.util.List;

public class KeyListener {
    final KeyEvent keyEvent;
    private final List<Integer> keys;
    public boolean active = true;
    public boolean passive = false;

    public KeyListener(int[] keys, KeyEvent keyEvent, boolean passive) {
        this.keys = Utils.toIntList(keys);
        this.keyEvent = keyEvent;
        this.passive = passive;
    }

    boolean listenedKey(int key) {
        return active && keys.contains(key);
    }

    public interface KeyEvent {
        void call(int charCode, int action);
    }
}
package net.comsoria.engine.view.input;

import net.comsoria.engine.view.Window;
import org.joml.Vector2d;
import org.lwjgl.system.MemoryUtil;

import java.nio.DoubleBuffer;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.GLFW_MOUSE_BUTTON_2;
import static org.lwjgl.glfw.GLFW.GLFW_PRESS;

public class MouseInput {
    private Vector2d cumulativePos = new Vector2d();
    private Vector2d lastPos = new Vector2d();
    private Vector2d movement = new Vector2d();

    private Vector2d start;

    private boolean inWindow = true;
    private boolean leftButtonPressed = false;
    private boolean rightButtonPressed = false;

    public boolean enabled = true;

    public void init(Window window) {
        DoubleBuffer x = MemoryUtil.memAllocDouble(1);
        DoubleBuffer y = MemoryUtil.memAllocDouble(1);
        glfwGetCursorPos(window.getWindowHandle(), x, y);
        start = new Vector2d(x.get(), y.get());

        glfwSetCursorPosCallback(window.getWindowHandle(), (windowHandle, xpos, ypos) -> {
            if (!enabled) return;

            Vector2d currentPos = new Vector2d(xpos - start.x(), ypos - start.y());
            movement = new Vector2d(currentPos.x - lastPos.x, currentPos.y - lastPos.y);

            cumulativePos.set(cumulativePos.x + movement.x, cumulativePos.y + movement.y);
            lastPos = currentPos;
        });

        glfwSetCursorEnterCallback(window.getWindowHandle(), (windowHandle, entered) -> {
            if (!enabled) return;

            inWindow = entered;
        });

        glfwSetMouseButtonCallback(window.getWindowHandle(), (windowHandle, button, action, mode) -> {
            if (!enabled) return;

            leftButtonPressed = button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS;
            rightButtonPressed = button == GLFW_MOUSE_BUTTON_2 && action == GLFW_PRESS;
        });
    }

    public void input() {
        movement.set(0, 0);
    }

    public Vector2d getMovementVec() {
        return movement;
    }

    public Vector2d getDisplVec() {
        return cumulativePos;
    }

    public boolean isLeftButtonPressed() {
        return leftButtonPressed;
    }

    public boolean isRightButtonPressed() {
        return rightButtonPressed;
    }
}
package net.comsoria.engine.view.light;

import net.comsoria.engine.view.color.Color3;
import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import org.joml.Vector3f;

public class DirectionalLight implements GLSLUniformBindable {
    public final Color3 color;
    public Vector3f direction;
    public float intensity;

    public DirectionalLight(Color3 color, Vector3f direction, float intensity) {
        this.color = color;
        this.direction = direction;
        this.intensity = intensity;
    }

    public DirectionalLight(DirectionalLight light) {
        this(light.color.clone(), new Vector3f(light.direction), light.intensity);
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name + ".color", this.color.getVec3());
        shaderProgram.setUniform(name + ".direction", this.direction);
        shaderProgram.setUniform(name + ".intensity", this.intensity);
    }

    public static void create(ShaderProgram shaderProgram, String name) {
        shaderProgram.createUniform(name + ".color");
        shaderProgram.createUniform(name + ".direction");
        shaderProgram.createUniform(name + ".intensity");
    }
}
package net.comsoria.engine.view.light;

import net.comsoria.engine.view.color.Color3;
import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import org.joml.Vector3f;

public class PointLight implements GLSLUniformBindable {
    public final Color3 color;
    public Vector3f position;
    public float intensity;
    public Attenuation attenuation;

    public PointLight(Color3 color, Vector3f position, float intensity) {
        attenuation = new Attenuation(1, 0, 0);
        this.color = color;
        this.position = position;
        this.intensity = intensity;
    }

    public PointLight(Color3 color, Vector3f position, float intensity, Attenuation attenuation) {
        this(color, position, intensity);
        this.attenuation = attenuation;
    }

    public PointLight(PointLight pointLight) {
        this(pointLight.color.clone(), new Vector3f(pointLight.position), pointLight.intensity, pointLight.attenuation);
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name + ".colour", this.color.getVec3());
        shaderProgram.setUniform(name + ".position", this.position);
        shaderProgram.setUniform(name + ".intensity", this.intensity);
        PointLight.Attenuation att = this.attenuation;
        shaderProgram. setUniform(name + ".att.constant", att.constant);
        shaderProgram.setUniform(name + ".att.linear", att.linear);
        shaderProgram.setUniform(name + ".att.exponent", att.exponent);
    }

    public static void create(ShaderProgram shaderProgram, String name) {
        shaderProgram.createUniform(name + ".colour");
        shaderProgram.createUniform(name + ".position");
        shaderProgram.createUniform(name + ".intensity");
        shaderProgram.createUniform(name + ".att.constant");
        shaderProgram.createUniform(name + ".att.linear");
        shaderProgram.createUniform(name + ".att.exponent");
    }

    public static class Attenuation {
        public float constant;
        public float linear;
        public float exponent;

        public Attenuation(float constant, float linear, float exponent) {
            this.constant = constant;
            this.linear = linear;
            this.exponent = exponent;
        }
    }
}
package net.comsoria.engine.view.light;

import net.comsoria.engine.view.color.Color3;

import java.util.ArrayList;
import java.util.List;

public class SceneLight {
    public final Color3 ambientLight = new Color3();
    public List<PointLight> pointLightList = new ArrayList<>();
    public List<SpotLight> spotLightList = new ArrayList<>();
    public DirectionalLight directionalLight;
}
package net.comsoria.engine.view.light;

import net.comsoria.engine.view.GLSL.GLSLUniformBindable;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import org.joml.Vector3f;

public class SpotLight implements GLSLUniformBindable {
    public PointLight pointLight;
    public Vector3f coneDirection;
    public float cutOff;

    public SpotLight(PointLight pointLight, Vector3f coneDirection, float cutOffAngle) {
        this.pointLight = pointLight;
        this.coneDirection = coneDirection;
        setCutOffAngle(cutOffAngle);
    }

    public SpotLight(SpotLight spotLight) {
        this(new PointLight(spotLight.pointLight), new Vector3f(spotLight.coneDirection), 0);
        this.cutOff = spotLight.cutOff;
    }

    public final void setCutOffAngle(float cutOffAngle) {
        this.cutOff = (float) Math.cos(Math.toRadians(cutOffAngle));
    }

    @Override
    public void set(ShaderProgram shaderProgram, String name) {
        shaderProgram.setUniform(name + ".pl", this.pointLight);
        shaderProgram.setUniform(name + ".conedir", this.coneDirection);
        shaderProgram.setUniform(name + ".cutoff", this.cutOff);
    }

    public static void create(ShaderProgram shaderProgram, String name) {
        PointLight.create(shaderProgram, name + ".pl");
        shaderProgram.createUniform(name + ".conedir");
        shaderProgram.createUniform(name + ".cutoff");
    }
}
package net.comsoria.engine.view;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.engine.view.GLSL.matrices.Transformation;

import java.io.Closeable;

public interface Renderable {
    Closeable render(Transformation transformation, Scene scene, RenderData renderData, Window window) throws Exception;
    void cleanup();
    boolean shouldRender();
    RenderOrder getRenderOrder();

    enum RenderOrder {
        First,
        Middle,
        End,
        Any
    }
}
package net.comsoria.engine.view;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.engine.view.GLSL.matrices.Transformation;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.opengl.GL11.*;

public class Renderer {
    public static void setViewPort(int x, int y, int width, int height) {
        glViewport(x, y, width, height);
    }

    public static void render(List<? extends Renderable> renderables, Scene scene, Transformation transformation, Window window) throws Exception {
        List<Closeable> toClose = new ArrayList<>();

        List<Renderable> start = new ArrayList<>();
        List<Renderable> middle = new ArrayList<>();
        List<Renderable> end = new ArrayList<>();
        for (Renderable renderable : renderables)
            switch (renderable.getRenderOrder()) {
                case First:
                    start.add(renderable);
                    break;
                case End:
                    end.add(renderable);
                    break;
                default:
                    middle.add(renderable);
                    break;
            }

        List<Renderable> total = new ArrayList<>(start);
        total.addAll(middle);
        total.addAll(end);

        for (Renderable gameItem : total) {
            if (!gameItem.shouldRender()) continue;

            Closeable item = gameItem.render(transformation, scene, RenderData.defaultRenderData, window);
            if (item != null && !toClose.contains(item)) toClose.add(item);
        }

        for (Closeable closeable : toClose) closeable.close();
    }
}
package net.comsoria.engine.view;

import net.comsoria.engine.view.color.Color3;
import net.comsoria.engine.view.color.Color4;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryUtil;

import java.nio.IntBuffer;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryUtil.NULL;

public class Window {
    private final String title;
    private int width, height;

    private boolean resized = false;

    private long windowHandle;

    public Window(String title, int width, int height) {
        this.title = title;

        this.width = width;
        this.height = height;
    }

    public void init() {
        System.setProperty("java.awt.headless", "true");

        GLFWErrorCallback.createPrint(System.err).set();

        if (!glfwInit()) throw new IllegalStateException("Unable to initialize GLFW");

        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
        glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//        glfwWindowHint(GLFW_FOCUSED, GL_TRUE);

        windowHandle = glfwCreateWindow(width, height, title, NULL, NULL);
        if (windowHandle == NULL) throw new RuntimeException("Failed to create the GLFW window");

        glfwSetFramebufferSizeCallback(windowHandle, (window, width, height) -> {
            this.width = width;
            this.height = height;
            this.setResized(true);
        });

        this.setPosition(0.5, 0.5);

        IntBuffer w = MemoryUtil.memAllocInt(1);
        IntBuffer h = MemoryUtil.memAllocInt(1);
        glfwGetFramebufferSize(windowHandle, w, h);
        this.width = w.get();
        this.height = h.get();
        this.setResized(true);

        glfwMakeContextCurrent(windowHandle);

        glfwSwapInterval(1);

        GL.createCapabilities();

        this.setClearColor(0, 0, 0, 1);

        glEnable(GL_DEPTH_TEST);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        //Enable for point lines
//        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    public void show() {
        glfwShowWindow(windowHandle);
        glfwFocusWindow(windowHandle);
    }

    public long getWindowHandle() {
        return windowHandle;
    }

    public void setClearColor(float r, float g, float b, float alpha) {
        glClearColor(r, g, b, alpha);
    }
    public void setClearColor(Color4 color) {
        setClearColor(color.r, color.g, color.b, color.a);
    }
    public void setClearColor(Color3 color) {
        setClearColor(color.r, color.g, color.b, 1.0f);
    }

    public static GLFWVidMode getVidMode() {
        return glfwGetVideoMode(glfwGetPrimaryMonitor());
    }

    public void setPosition(double xP, double yP) { //Percentages
        GLFWVidMode vidmode = getVidMode();
        int x = (int) ((vidmode.width() - width) * xP);
        int y = (int) ((vidmode.height() - height) * yP);

        glfwSetWindowPos(windowHandle, x, y);
    }

    public void setResized(boolean resized) {
        this.resized = resized;
    }

    public boolean isResized() {
        return resized;
    }

    public boolean windowShouldClose() {
        return glfwWindowShouldClose(windowHandle);
    }

    public void update() {
        glfwSwapBuffers(windowHandle);
        glfwPollEvents();
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public void hideCursor() {
        glfwSetInputMode(this.windowHandle, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }

    public void showCursor() {
        glfwSetInputMode(this.windowHandle, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    public void setMousePos(double x, double y) {
        glfwSetCursorPos(this.windowHandle, x, y);
    }
}
package net.comsoria.game.physics;

public class Gravity {
    private final int strength;

    public Gravity(int strength) {
        this.strength = strength;
    }

    public void getNewHeight() {

    }
}
package net.comsoria.game.physics;

public class Velocity {
}
package net.comsoria.game;

import org.joml.Vector2f;
import org.joml.Vector3f;

public class Player {
    private Vector3f position;

    private float speed = 10;
    public float sprintMultiplier = 10;

    public Player(Vector3f position) {
        this.position = position;
    }

    public Vector3f getPosition() {
        return position;
    }

    public void setPosition(Vector3f position) {
        this.position = position;
    }

    public Vector2f get2DPosition() {
        return new Vector2f(position.x, position.z);
    }

    public float getSpeed(boolean sprint) {
        return speed * (sprint? sprintMultiplier:1);
    }
}
package net.comsoria.game;

import net.comsoria.engine.Scene;
import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.loaders.OBJLoader;
import net.comsoria.engine.view.GLSL.programs.CustomShaderProgram;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.graph.BufferAttribute;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.Texture;
import net.comsoria.engine.view.graph.mesh.Mesh;
import net.comsoria.engine.view.graph.mesh.SkyBox;
import org.joml.Matrix4f;
import org.joml.Vector3f;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

public class SkyDome {
    public static Mesh genSkyDome(String fragment, String vertex, float size, Texture sun) throws IOException {
        Tuple<List<BufferAttribute>, int[]> data = OBJLoader.loadGeometry(Utils.utils.p("$models/skydome.obj"));
        data.getA().remove(1);
        data.getA().remove(1);
        Mesh dome = new SkyBox(new Geometry(data), new Material(), null);

        dome.shaderProgram = new CustomShaderProgram(fragment, vertex, Arrays.asList("color1", "color2", "modelViewMatrix", "projectionMatrix", "sunDirection", "sunLine"), Arrays.asList("sun")) {
            @Override
            public void setupScene(Scene scene, Transformation transformation) {
                this.setUniform("projectionMatrix", transformation.getProjection());
                Vector3f direction = scene.light.directionalLight.direction;
                this.setUniform("sunDirection", direction);
//                shaderProgram.setUniform("sunLine", new Circle().getTangent((float) Math.atan2(direction.x, direction.y)));

                this.setUniform("color1", scene.sky.getMainColor().getVec3());
                this.setUniform("color2", scene.sky.getSecondColor().getVec3());
            }

            @Override
            public void setupMesh(Mesh mesh, Matrix4f modelMatrix, Transformation transformation) {
                this.setUniform("modelViewMatrix", modelMatrix);
            }
        };

        dome.scale = size;
        dome.initShaderProgram();
        dome.material.textures.add(sun);

        return dome;
    }
}
package net.comsoria.game.terrain;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.Renderable;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.game.terrain.terrainFeature.TerrainFeature;
import net.comsoria.game.terrain.terrainFeature.TerrainFeatureLoader;
import org.joml.Vector2f;
import org.joml.Vector2i;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Chunk {
    public Vector2i position;
    public List<TerrainFeature> terrainFeatures = new ArrayList<>();

    public Chunk(Vector2i position) {
        this.position = position;
    }

    void load(List<TerrainFeatureLoader> terrainFeatureLoaders, float scale) throws IOException {
        for (TerrainFeatureLoader loader : terrainFeatureLoaders) {
            terrainFeatures.addAll(loader.load(this.position, scale));
        }
    }

    void hide() {
        for (TerrainFeature feature : this.terrainFeatures) {
            feature.getGameObject().visible = false;
        }
    }

    void show() {
        for (TerrainFeature feature : this.terrainFeatures) {
            feature.getGameObject().visible = true;
        }
    }

    boolean isShown() {
        if (terrainFeatures.size() > 0) return terrainFeatures.get(0).getGameObject().visible;

        return true;
    }
}
package net.comsoria.game.terrain.terrainFeature.cave.cave;

import net.comsoria.engine.loaders.Shape;
import net.comsoria.engine.utils.random.Random;
import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.graph.BufferAttribute;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.mesh.Mesh;
import net.comsoria.engine.view.graph.mesh.NoViewMatrixMesh;
import net.comsoria.game.terrain.terrainFeature.TerrainFeature;
import net.comsoria.game.terrain.terrainFeature.cave.cave.generation.CaveTerrainGenerator;
import org.joml.Vector2f;
import org.joml.Vector3f;

import java.io.IOException;
import java.util.List;

import static org.lwjgl.opengl.GL11.GL_BACK;
import static org.lwjgl.opengl.GL11.GL_FRONT;

public class Cave implements TerrainFeature {
    private Mesh mesh;
    private Vector3f position;
    private final float radius;
    private final int size;

    public Cave(int size, Vector3f position, float radius) {
        this.size = size;
        this.position = position;
        this.radius = radius;
    }

    public void loadGameObject(ShaderProgram shaderProgram, CaveTerrainGenerator generator) throws IOException {
        Tuple<List<BufferAttribute>, int[]> parts = Shape.genSphere(size, radius);
//        parts.getA().add(Shape.generateSphereDisplacement(parts.getA().get(0), 0.3f, 3, radius));

        generator.updateBuffer(parts.getA().get(0));

        mesh = new NoViewMatrixMesh(new Geometry(parts), new Material(), shaderProgram);
//        mesh.geometry.setCullFace(GL_FRONT);

        mesh.position.set(position);
        mesh.scale = radius * 2;
    }

    @Override public Mesh getGameObject() {
        return mesh;
    }

    public Vector3f getPosition() {
        return position;
    }

    public float getRadius() {
        return radius;
    }

    public int getSize() {
        return size;
    }
}
package net.comsoria.game.terrain.terrainFeature.cave.cave;

import net.comsoria.engine.loaders.GLSLLoader;
import net.comsoria.engine.utils.random.NoiseGenerator;
import net.comsoria.engine.utils.random.Random;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.GLSL.programs.FadeOutShaderProgram;
import net.comsoria.engine.view.batch.BatchRenderType;
import net.comsoria.engine.view.batch.BatchRenderer;
import net.comsoria.game.terrain.terrainFeature.TerrainFeature;
import net.comsoria.game.terrain.terrainFeature.TerrainFeatureLoader;
import net.comsoria.game.terrain.terrainFeature.cave.cave.generation.CaveTerrainGenerator;
import org.joml.Vector2f;
import org.joml.Vector2i;
import org.joml.Vector3f;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class CaveLoader implements TerrainFeatureLoader {
    private final float positionSeed;
    private final float sizeSeed;
    private final float iterationSeed;
    private final int maxPerChunk;
    private final CaveTerrainGenerator generator;
    private final int maxSize;
    private final float maxRealSize;
    private BatchRenderer batchRenderer = new BatchRenderer(new BatchRenderType());

    public CaveLoader(float skyDomeR, float positionSeed, float sizeSeed, float iterationSeed, int maxPerChunk, CaveTerrainGenerator generator, int maxSize, float maxRealSize) throws IOException {
        this.positionSeed = positionSeed;
        this.sizeSeed = sizeSeed;
        this.iterationSeed = iterationSeed;
        this.maxPerChunk = maxPerChunk;
        this.generator = generator;
        this.maxSize = maxSize;
        this.maxRealSize = maxRealSize;

        Map<String, String> constants = Utils.buildMap("skyDomeRadius", String.valueOf(skyDomeR), "width", String.valueOf(2));
        batchRenderer.batchRenderType.shaderProgram = new FadeOutShaderProgram(
                GLSLLoader.loadGLSL(Utils.utils.p("$shaders/underground/chunk_vertex.v.glsl"), constants),
                GLSLLoader.loadGLSL(Utils.utils.p("$shaders/underground/chunk_fragment.f.glsl"), constants)
        );

        batchRenderer.batchRenderType.shaderProgram.init();
    }

    @Override
    public List<TerrainFeature> load(Vector2i chunkPosition, float scale) throws IOException {
        List<TerrainFeature> result = new ArrayList<>();

        float x = chunkPosition.x + 78.23f;
        float y = chunkPosition.y + 12.56f;

        int iterations = (int) (((Random.random.noise(x, y, 87.92f * iterationSeed) + 1) * 0.5) * maxPerChunk);

        for (int i = 0; i < iterations; i++) {
            float radius = (Random.random.noise(x, y,  54.27f * sizeSeed * (i + 34.23f)) + 1) * 0.5f;

            Cave cave = new Cave(
                    2 * (Math.round((int) (radius * maxSize) / 2)),
                    new Vector3f(
                            (Random.random.noise((float) i + 83.54f, x, y, 34.73f * positionSeed) + chunkPosition.x) * scale,
                            ((Random.random.noise((float) i + 73.23f, x, y, 62.93f * positionSeed) - 1) * 0.5f) * scale,
                            (Random.random.noise((float) i + 92.34f, x, y, 12.34f * positionSeed) + chunkPosition.y) * scale
                    ),
                    radius * maxRealSize * scale

            );
            cave.loadGameObject(batchRenderer.batchRenderType.shaderProgram, generator);
            batchRenderer.gameObjects.add(cave.getGameObject());
            result.add(cave);
        }

        return result;
    }

    @Override
    public BatchRenderer getBatchRenderer() {
        return batchRenderer;
    }
}
package net.comsoria.game.terrain.terrainFeature.cave.cave.generation;

import net.comsoria.engine.utils.SimplexNoise;
import net.comsoria.engine.view.graph.BufferAttribute;
import net.comsoria.game.terrain.terrainFeature.Octave;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class CaveOctaveGenerator implements CaveTerrainGenerator {
    public List<Octave> octaves;
    public float overallHeight = 1;
    public float overallMultiplier = 1;

    public CaveOctaveGenerator(List<Octave> octaves) {
        this.octaves = octaves;
    }

    public CaveOctaveGenerator(List<Octave> octaves, float seed) {
        for (Octave octave : octaves) {
            octave.seed = seed;
        }
        this.octaves = octaves;
    }

    public CaveOctaveGenerator(int octaves, float seed) {
        this.octaves = new ArrayList<>();
        for (int i = 0; i < octaves; i++) {
            Octave octave = new Octave();
            octave.height = (float) (1 / (Math.pow(2, i)));
            octave.multiplier = (float) Math.pow(2, i);
            octave.seed = seed;
            this.octaves.add(octave);
        }
    }

    public CaveOctaveGenerator(int octaves, float seed, float overallMultiplier, float overallHeight) {
        this(octaves, seed);
        this.overallHeight = overallHeight;
        this.overallMultiplier = overallMultiplier;
    }

    public float get(float x, float y) {
        float value = 0;
        for (Octave octave : octaves) {
            value += SimplexNoise.noise(x * octave.multiplier, y * octave.multiplier, octave.seed) * octave.height;
        }
        return value;
    }


    public void updateBuffer(BufferAttribute positions) {
        for (Octave octave : octaves) {
            for (int i = 0; i < positions.parts(); i++) {
                Vector3f seed = positions.getVec3(i).mul(octave.multiplier);

                positions.add(i * 3,
                        (float) SimplexNoise.noise(seed.x, seed.y, seed.z, octave.seed * 45.23f) * octave.height
                );
                positions.add((i * 3) + 1,
                        (float) SimplexNoise.noise(seed.x, seed.y, seed.z, octave.seed * 23.21f) * octave.height
                );
                positions.add((i * 3) + 2,
                        (float) SimplexNoise.noise(seed.x, seed.y, seed.z, octave.seed * 78.67f) * octave.height
                );
            }
        }
    }
}
package net.comsoria.game.terrain.terrainFeature.cave.cave.generation;

import net.comsoria.engine.view.graph.BufferAttribute;

public interface CaveTerrainGenerator {
    void updateBuffer(BufferAttribute bufferAttribute);
}
package net.comsoria.game.terrain.terrainFeature.cave.tunnel;

import net.comsoria.engine.loaders.Shape;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.mesh.Mesh;
import net.comsoria.game.terrain.terrainFeature.TerrainFeature;
import org.joml.Vector2f;

import java.io.IOException;

public class CaveTunnel implements TerrainFeature {
    private Vector2f position;
    private Mesh mesh;

    public CaveTunnel(Vector2f position) {
        this.position = position;
    }

    public void loadGameObject(int graphicalSize, int range, ShaderProgram shaderProgram) throws IOException {
        this.mesh = new Mesh(new Geometry(Shape.genCylinder(10, 20, 1)), new Material(), shaderProgram);
    }

    @Override
    public Mesh getGameObject() {
        return null;
    }


    public Vector2f getPosition() {
        return null;
    }
}
//package net.comsoria.game.terrain.terrainFeature.cave.tunnel;
//
//import net.comsoria.engine.utils.Utils;
//import net.comsoria.engine.view.batch.BatchRenderer;
//import net.comsoria.game.terrain.World;
//import net.comsoria.game.terrain.terrainFeature.TerrainFeatureLoader;
//import org.joml.Vector2f;
//
//public class CaveTunnelLoader implements TerrainFeatureLoader {
//    private float probability;
//
//    public CaveTunnelLoader(float probability) {
//        this.probability = probability;
//    }
//
//    @Override
//    public void updateAroundPlayer(Vector2f position, World world, int radius) throws Exception {
//        position = Utils.round(position, 0);
//
//
//    }
//
//    @Override
//    public BatchRenderer getBatchRenderer() {
//        return null;
//    }
//}
package net.comsoria.game.terrain.terrainFeature;

public class Octave {
    public Float height = null;
    public Float multiplier = null;
    public Float seed = null;

    public Octave() {}

    public Octave(float multiplier, float height) {
        this.height = height;
        this.multiplier = multiplier;
    }

    public Octave(float multiplier, float height, float seed) {
        this.height = height;
        this.multiplier = multiplier;
        this.seed = seed;
    }
}
package net.comsoria.game.terrain.terrainFeature.surfaceChunk.generation;

import net.comsoria.engine.utils.Grid;
import net.comsoria.engine.utils.SimplexNoise;
import net.comsoria.game.terrain.terrainFeature.Octave;
import org.joml.Vector2i;

import java.util.ArrayList;
import java.util.List;

public class SurfaceChunkOctaveGenerator implements SurfaceChunkTerrainGenerator {
    public List<Octave> octaves;
    public float overallHeight = 1;
    public float overallMultiplier = 1;

    public SurfaceChunkOctaveGenerator(List<Octave> octaves) {
        this.octaves = octaves;
    }

    public SurfaceChunkOctaveGenerator(List<Octave> octaves, float seed) {
        for (Octave octave : octaves) {
            octave.seed = seed;
        }
        this.octaves = octaves;
    }

    public SurfaceChunkOctaveGenerator(int octaves, float seed) {
        this.octaves = new ArrayList<>();
        for (int i = 0; i < octaves; i++) {
            Octave octave = new Octave();
            octave.height = (float) (1 / (Math.pow(2, i)));
            octave.multiplier = (float) Math.pow(2, i);
            octave.seed = seed;
            this.octaves.add(octave);
        }
    }

    public SurfaceChunkOctaveGenerator(int octaves, float seed, float overallMultiplier, float overallHeight) {
        this(octaves, seed);
        this.overallHeight = overallHeight;
        this.overallMultiplier = overallMultiplier;
    }

    public float get(float x, float y) {
        float value = 0;
        for (Octave octave : octaves) {
            value += SimplexNoise.noise(x * octave.multiplier, y * octave.multiplier, octave.seed) * octave.height;
        }
        return value;
    }

    @Override
    public void updateGrid(Grid<Float> grid, Vector2i chunkPosition) {
        for (int x = 0; x < grid.getWidth(); x++) {
            for (int y = 0; y < grid.getHeight(); y++) {
                grid.set(x, y, this.get(
                        (x + (chunkPosition.x * grid.getWidth()) - chunkPosition.x) * this.overallMultiplier,
                        (y + (chunkPosition.y * grid.getHeight()) - chunkPosition.y) * this.overallMultiplier
                ) * this.overallHeight);
            }
        }
    }
}
package net.comsoria.game.terrain.terrainFeature.surfaceChunk.generation;

import net.comsoria.engine.utils.Grid;
import org.joml.Vector2i;

public interface SurfaceChunkTerrainGenerator {
    void updateGrid(Grid<Float> grid, Vector2i chunkPosition);
}
package net.comsoria.game.terrain.terrainFeature.surfaceChunk;

import net.comsoria.engine.utils.Grid;
import net.comsoria.engine.utils.random.Random;
import net.comsoria.engine.utils.Tuple;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.loaders.OBJLoader;
import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.graph.BufferAttribute;
import net.comsoria.engine.view.graph.Geometry;
import net.comsoria.engine.view.graph.Material;
import net.comsoria.engine.view.graph.mesh.Mesh;
import net.comsoria.engine.view.graph.mesh.NoViewMatrixMesh;
import net.comsoria.game.terrain.terrainFeature.TerrainFeature;
import org.joml.Vector2f;
import org.joml.Vector2i;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.lwjgl.opengl.GL11.GL_BACK;

public class SurfaceChunk implements TerrainFeature {
    private static Tuple<List<BufferAttribute>, int[]> vertices;

    private final Grid<Float> grid;
    private final Vector2i chunkPosition;
    private Vector2f position;
    private Mesh gameObject;

    public SurfaceChunk(Grid<Float> grid, Vector2i position) {
        this.grid = grid;
        this.chunkPosition = position;
        this.position = new Vector2f(position);
    }

    static {
        try {
            vertices = OBJLoader.loadGeometry(Utils.utils.p("$models/chunk_plane.obj"));
            vertices.getA().remove(1);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("ERROR");
        }
    }

    private static Tuple<List<BufferAttribute>, int[]> getVertices() {
        Tuple<List<BufferAttribute>, int[]> result = new Tuple<>();
        result.setA(new ArrayList<>(Arrays.asList(vertices.getA().get(0).clone())));
        result.setB(vertices.getB());
        return result;
    }

    public void loadGameObject(float graphicalSize, ShaderProgram shaderProgram) throws IOException {
        this.position = this.position.mul(graphicalSize);

        Tuple<List<BufferAttribute>, int[]> data = getVertices();

        Float[] array = grid.getArray(Float.class);
        float[] displacement = new float[grid.getHeight() * grid.getWidth() * 2];

        Vector2i relative = new Vector2i((chunkPosition.x * grid.getWidth()) - chunkPosition.x, (chunkPosition.y * grid.getHeight()) - chunkPosition.y);

        for (int i = 0; i < array.length; i++) {
            Vector2i vec = grid.getXY(i).add(relative);

            displacement[i * 2] = Random.random.noise(vec.x, vec.y, 10) * 0.005f;
            displacement[(i * 2) + 1] = Random.random.noise(vec.x, vec.y, 76) * 0.005f;

            data.getA().get(0).set((i * 3) + 1, array[i]);
        }

        data.getA().add(new BufferAttribute(displacement, 2));

        gameObject = new NoViewMatrixMesh(new Geometry(data), new Material(), shaderProgram);

        gameObject.material.ambientColour.set(0, 0, 0, 0);
        gameObject.geometry.setCullFace(GL_BACK);

        gameObject.position.set(position.x, 0, position.y);
        gameObject.scale = graphicalSize;
    }

    @Override
    public Mesh getGameObject() {
        return gameObject;
    }
}
package net.comsoria.game.terrain.terrainFeature.surfaceChunk;

import net.comsoria.game.terrain.terrainFeature.TerrainFeature;

public class SurfaceChunk2 implements TerrainFeature {
    @Override
    public boolean getBlock(int x, int y) {
        return false;
    }
}
package net.comsoria.game.terrain.terrainFeature.surfaceChunk;

import net.comsoria.engine.loaders.GLSLLoader;
import net.comsoria.engine.utils.Grid;
import net.comsoria.engine.utils.Utils;
import net.comsoria.engine.view.GLSL.programs.FadeOutShaderProgram;
import net.comsoria.engine.view.batch.BatchRenderType;
import net.comsoria.engine.view.batch.BatchRenderer;
import net.comsoria.game.terrain.terrainFeature.TerrainFeature;
import net.comsoria.game.terrain.terrainFeature.TerrainFeatureLoader;
import net.comsoria.game.terrain.terrainFeature.surfaceChunk.generation.SurfaceChunkTerrainGenerator;
import org.joml.Vector2i;

import java.io.IOException;
import java.util.*;

public class SurfaceChunkLoader implements TerrainFeatureLoader {
    private final SurfaceChunkTerrainGenerator generator;
    private final BatchRenderer batchRenderer = new BatchRenderer(new BatchRenderType());
    private final int chunkSize;

    public SurfaceChunkLoader(SurfaceChunkTerrainGenerator generator, int chunkSize, float skyDomeRad) throws IOException {
        this.generator = generator;

        this.chunkSize = chunkSize;

        Map<String, String> constants = Utils.buildMap(
                "skyDomeRadius", String.valueOf(skyDomeRad),
                "width", String.valueOf(this.chunkSize - 1)
        );

        batchRenderer.batchRenderType.shaderProgram = new FadeOutShaderProgram(
                GLSLLoader.loadGLSL(Utils.utils.p("$shaders/chunk/chunk_vertex.v.glsl"), constants),
                GLSLLoader.loadGLSL(Utils.utils.p("$shaders/chunk/chunk_fragment.f.glsl"), constants)
        );
        batchRenderer.batchRenderType.shaderProgram.init();
    }

    @Override
    public List<TerrainFeature> load(Vector2i chunkPosition, float scale) throws IOException {
        Grid<Float> grid = new Grid<>(this.chunkSize, this.chunkSize);
        generator.updateGrid(grid, chunkPosition);

        SurfaceChunk chunk = new SurfaceChunk(grid, chunkPosition);
        chunk.loadGameObject(scale, this.batchRenderer.batchRenderType.shaderProgram);
        this.batchRenderer.gameObjects.add(chunk.getGameObject());

        return Collections.singletonList(chunk);
    }

    @Override
    public BatchRenderer getBatchRenderer() {
        return batchRenderer;
    }
}
package net.comsoria.game.terrain.terrainFeature;

import net.comsoria.engine.view.GLSL.ShaderProgram;
import net.comsoria.engine.view.graph.mesh.Mesh;
import org.joml.Vector2f;

import java.io.IOException;

public interface TerrainFeature {
//    Mesh getGameObject();
    boolean getBlock(int x, int y);
}
package net.comsoria.game.terrain.terrainFeature;

import net.comsoria.engine.view.batch.BatchRenderer;
import net.comsoria.game.terrain.World;
import org.joml.Vector2f;
import org.joml.Vector2i;

import java.io.IOException;
import java.util.List;

public interface TerrainFeatureLoader {
    List<TerrainFeature> load(Vector2i chunkPosition, float scale) throws IOException;
    BatchRenderer getBatchRenderer();
}
package net.comsoria.game.terrain;

import net.comsoria.engine.Scene;
import net.comsoria.engine.view.GLSL.matrices.Transformation;
import net.comsoria.engine.view.Renderable;
import net.comsoria.engine.view.Window;
import net.comsoria.engine.view.batch.BatchCloseable;
import net.comsoria.engine.view.batch.RenderData;
import net.comsoria.game.terrain.terrainFeature.TerrainFeature;
import net.comsoria.game.terrain.terrainFeature.TerrainFeatureLoader;
import net.comsoria.game.terrain.terrainFeature.surfaceChunk.SurfaceChunk;
import org.joml.Vector2f;
import org.joml.Vector2i;
import org.joml.Vector3f;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class World implements Renderable {
    public final static int MAX_DEPTH = 20000;

    private final List<TerrainFeatureLoader> loaders = new ArrayList<>();
    private List<Chunk> chunks = new ArrayList<>();

    private int radius = 4;
    private float scale;

    public World(float scale) {
        this.scale = scale;
    }

    public void addLoader(TerrainFeatureLoader loader) {
        this.loaders.add(loader);
    }

    public Chunk loadChunk(Vector2i position) throws IOException {
        Chunk chunk = new Chunk(position);
        chunk.load(loaders, scale);
        return chunk;
    }

    public Chunk getChunk(Vector2i position) {
        for (Chunk chunk : chunks)
            if (position.x == chunk.position.x && position.y == chunk.position.y) return chunk;

        return null;
    }

    public void updateAroundPlayer(Vector2f playerPosition) throws IOException {
        playerPosition = playerPosition.mul(1 / scale);

        Vector2i chunkPosition = new Vector2i((int) playerPosition.x, (int) playerPosition.y);

        for (int x = -radius; x < radius; x++) {
            for (int y = -radius; y < radius; y++) {
                Vector2i relativePosition = new Vector2i(x + chunkPosition.x, y + chunkPosition.y);

                Chunk existing = this.getChunk(relativePosition);

                if (existing == null) {
                    Chunk chunk = this.loadChunk(relativePosition);
                    this.chunks.add(chunk);
                } else if (!existing.isShown()) existing.show();
            }
        }

        for (Chunk chunk : this.chunks) {
            if (chunk.isShown() && new Vector2f(chunk.position).distance(playerPosition) > radius) {
                chunk.hide();
            }
        }
    }

    @Override
    public Closeable render(Transformation transformation, Scene scene, RenderData renderData, Window window) throws Exception {
        List<Closeable> toClose = new ArrayList<>();

        for (TerrainFeatureLoader loader : loaders) {
            toClose.add(loader.getBatchRenderer().render(transformation, scene, renderData, window));
        }

        return new BatchCloseable(toClose);
    }

    @Override
    public void cleanup() {
        for (TerrainFeatureLoader loader : loaders)
            loader.getBatchRenderer().cleanup();
    }

    @Override
    public boolean shouldRender() {
        return true;
    }

    @Override
    public RenderOrder getRenderOrder() {
        return RenderOrder.Any;
    }
}

