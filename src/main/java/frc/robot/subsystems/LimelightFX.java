package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;

import java.lang.reflect.Constructor;
import java.util.*;
import java.util.function.*;

public final class LimelightFX extends SubsystemBase {
	/** The maximum number of modules the FX supports. */
	public static final int maxModules = 32;
	/** The number of layers the FX supports per module. */
	public static final int maxLayers = 2;
	/** The number of audio channels the FX supports. */
	public static final int maxChannels = 2;
	/** The maximum number of total LEDs the FX supports. */
	public static final int maxLEDs = 1024;
	/** The longest a file path can be. Shorter paths conserves memory. */
	public static final int maxFilepath = 32;
	/** LED framerate. */
	public static final int fps = 33;

	private static final Function<Double, String> time = time -> Integer.toString((int) (time * LimelightFX.fps));
	private static final Function<
		Double,
		String> speed = time -> Integer.toString((int) (time * LimelightFX.fps * 255));

	/** An RGBA color. Components are 0-1. */
	public static final class Color {
		public static final Color black = new Color(0);
		public static final Color WHITE = new Color(1);
		public static final Color RED = new Color(1, 0, 0);
		public static final Color GREEN = new Color(0, 1, 0);
		public static final Color BLUE = new Color(0, 0, 1);
		public static final Color ORANGE = new Color(1, 0.4, 0);

		public final double a;
		public final double r;
		public final double g;
		public final double b;

		public Color(final double r, final double g, final double b, final double a) {
			this.a = a;
			this.r = r;
			this.g = g;
			this.b = b;
		}

		public Color(final double r, final double g, final double b) { this(r, g, b, 1); }

		public Color(final double value) { this(value, value, value); }

		@Override
		public final boolean equals(final Object other) {
			if(this != other) return false;
			if(this instanceof Color) {
				final Color col = (Color) other;
				return this.r == col.r && this.g == col.g && this.b == col.b;
			} else if(other instanceof final int[] arr) {
				return this.r == arr[0] && this.g == arr[1] && this.b == arr[2];
			}
			return false;
		}

		@Override
		public final String toString() {
			return String
				.format(
					"0x%02x%02x%02x%02x",
					(int) Math.min(Math.max(Math.floor(this.a * 255), 0), 255),
					(int) Math.min(Math.max(Math.floor(this.r * 255), 0), 255),
					(int) Math.min(Math.max(Math.floor(this.g * 255), 0), 255),
					(int) Math.min(Math.max(Math.floor(this.b * 255), 0), 255)
				);
		}
	}

	/** An audio tone waveform. */
	public static enum WaveForm {
		Sine(0), Square(1), Sawtooth(2);

		private WaveForm(final int id) { this.id = id; }

		public final int id;
	}

	/** Built-in sounds that can be played directly. */
	public static enum SystemSound {
		Beep0(0),
		Beep1(1),
		Beep2(2),
		Beep3(3),
		Beep4(4),
		Buzz0(5),
		Buzz1(6),
		Buzz2(7),
		Buzz3(8),
		Buzz4(9),
		Explosion(10),
		MenuMove(11),
		MonsterAppear(12),
		MonsterHurt(13),
		Coin(14),
		Swoosh(15),
		Damage(16),
		DogBark(17),
		CarHorn0(18),
		CarHorn1(19);

		private SystemSound(final int id) { this.id = id; }

		public final int id;
	}

	/** An LED module. */
	public static final class Module {
		/** Defines the shape of the module. */
		public static final class Geometry {
			public static enum Kind {
				Strip(64), Grid(65), Ring(66);

				private Kind(final int id) { this.id = id; }

				public final int id;
			}

			public static final Geometry strip = new Geometry(Kind.Strip, 16, 1);
			public static final Geometry grid = new Geometry(Kind.Grid, 24, 12);
			public static final Geometry ring = new Geometry(Kind.Ring, /* unknown */ 0, /* unknown */ 0);

			private Geometry(final Kind kind, final int width, final int height) {
				this.kind = kind;
				this.width = width;
				this.height = height;
			}

			public final Kind kind;
			public final int width;
			public final int height;

			public final Geometry size(final int width, final int height) {
				return new Geometry(this.kind, width, height);
			}
		}

		/** Defines the rotation of the module. */
		public static enum Rotation {
			R0(0), R90(1), R180(2), R270(3);

			private Rotation(final int id) { this.id = id; }

			public final int id;
		}

		private Module(final int id, final LimelightFX fx, final Geometry geo, final Rotation rot) {
			this.id = id;
			this.fx = fx;

			this.geo = geo;
			this.rot = rot;
		}

		public final int id;
		public final LimelightFX fx;

		public final Geometry geo;
		public final Rotation rot;

		private final Layer[] layers = new Layer[LimelightFX.maxLayers];
	}

	/** Stores layer data. */
	private static final class Layer {
		Layer(final int id) { this.id = id; }

		/** Numerical index. */
		public final int id;

		/** Behaviors on this layer. */
		private final HashSet<Behavior<?>> behaviors = new HashSet<>();
		/** Whether or not this layer had an active behavior last update. */
		private volatile boolean wasActive = false;
	}

	/** A behavior on a module and layer. */
	public static abstract class Behavior<T extends Behavior<T>> {
		/** A parameter that can be updated to the FX. */
		public static final class Param<T> {
			private static final class Storage<T> {
				private Storage(final LimelightFX fx) { this.fx = fx; }

				/** The internal value. */
				public volatile T value;
				/** The dirty flag, if true, the parameter will be sent next update. */
				public volatile boolean dirty = true;

				/** The LimelightFX this parameter is from. */
				public final LimelightFX fx;
			}

			/**
			 * Initialize the parameter.
			 *
			 * @param fx
			 *                    The LimelightFX this parameter is from.
			 * @param initial
			 *                    The initial value of the parameter.
			 * @param trans
			 *                    The transformer function, defines how the parameter is sent.
			 */
			private Param(final LimelightFX fx, final T initial, final Function<T, String> trans) {
				this.storage = new Storage<>(fx);
				this.storage.value = initial;
				this.trans = trans;
			}

			private Param(final LimelightFX fx, final T initial) { this(fx, initial, T::toString); }

			private volatile Storage<T> storage;

			/** The transformer function, defines how the parameter is sent. */
			public final Function<T, String> trans;

			/** Get the value of the parameter. */
			public synchronized final T get() { return this.storage.value; }

			/**
			 * Set the value of the parameter and mark it dirty if the value is not equal.
			 *
			 * @param value
			 *                  The value to set.
			 */
			public synchronized final boolean set(final T value) {
				if(this.storage.value.equals(value)) return false;
				this.storage.value = value;
				this.storage.dirty = true;
				return true;
			}

			/** Whether or not this parameter is dirty. */
			public synchronized final boolean dirty() {
				return this.storage.dirty;
			}

			/** Mark this parameter dirty without mutating the value. */
			public synchronized final void forceDirty() {
				this.storage.dirty = true;
			}

			/** Clear the dirty parameter value. */
			public synchronized final void clearDirty() {
				this.storage.dirty = false;
			}

			/**
			 * Link this parameter and another together. Changes on either reflect on both, and can be linked with other parameters. The value of this parameter will be overriden
			 *
			 * @param from
			 *                 The parameter to copy from.
			 */
			public synchronized final void link(final Param<T> from) {
				if(this.storage.fx.initialized)
					throw new Error(
						"[ERROR] LimelightFX: Cannot create/modify Behaviors after initialization is complete"
					);
				this.storage = from.storage;
			}

			@Override
			public synchronized final String toString() { return this.trans.apply(this.storage.value); }
		}

		public static enum Kind {
			Null(-1),
			SolidColor(0),
			Gradient(1),
			Blink(2),
			Scroll(3),
			Cylon(4),
			Emoji(5),
			Chevrons(6),
			Shapes(7),
			SwerveRing(8),
			BarGraph(9),
			AudioWave(10),
			SlotMachine(11),
			CountdownTimer(12),
			TeamNumber(13),
			BubbleLevel(14),
			Image(15),
			Video(16),
			LineGraph(17),
			SlideShow(18);

			private Kind(final int id) { this.id = id; }

			public final int id;
		}

		public static enum Direction {
			Up(0), Right(1), Down(2), Left(3);

			private Direction(final int id) { this.id = id; }

			public final int id;
		}

		/** Single color with a fade time. */
		public static class SolidColor extends Behavior<SolidColor> {
			SolidColor(final LimelightFX fx, final Module module, final int layer) {
				super(Kind.Blink, fx, module, layer);

				this.params = new Param<?>[] { this.color, this.fade, };
			}

			/** The color. */
			public final Param<Color> color = new Param<>(this.fx, Color.RED);

			/** The time to fade between colors. (s) */
			public final Param<Double> fade = new Param<>(this.fx, 0.0, LimelightFX.time);
		}

		/** Rainbow. */
		public static class Gradient extends Behavior<Gradient> {
			Gradient(final LimelightFX fx, final Module module, final int layer) {
				super(Kind.Gradient, fx, module, layer);

				this.params = new Param<?>[] { this.colorA, this.colorB, this.fade, this.dir };
			}

			public final Param<Color> colorA = new Param<>(this.fx, new Color(255, 0, 0));
			public final Param<Color> colorB = new Param<>(this.fx, new Color(255, 0, 0));

			public final Param<Double> fade = new Param<>(this.fx, 0.0, LimelightFX.time);
			public final Param<Direction> dir = new Param<>(this.fx, Direction.Up);
		}

		/** A simple configurable blink pattern. */
		public static class Blink extends Behavior<Blink> {
			Blink(final LimelightFX fx, final Module module, final int layer) {
				super(Kind.Blink, fx, module, layer);

				this.params = new Param<?>[] {
					this.colorA,
					this.colorB,

					this.timeOnA,
					this.timeOffA,
					this.timeBetween,
					this.timeOnB,
					this.timeOffB,
					this.timeRepeat,

					this.blinkCountA,
					this.blinkCountB,
					this.repeatCount,

					this.fadeInA,
					this.fadeOutA,
					this.fadeInB,
					this.fadeOutB, };
			}

			/** The A color. */
			public final Param<Color> colorA = new Param<>(this.fx, new Color(255, 0, 0));
			/** The B color. */
			public final Param<Color> colorB = new Param<>(this.fx, new Color(0, 255, 0));

			/** The time color A should be on for. (s) */
			public final Param<Double> timeOnA = new Param<>(this.fx, 0.25, LimelightFX.time);
			/** The time color A should be off for. (s) */
			public final Param<Double> timeOffA = new Param<>(this.fx, 0.25, LimelightFX.time);
			/** The time between color A and color B. (s) */
			public final Param<Double> timeBetween = new Param<>(this.fx, 1.0, LimelightFX.time);
			/** The time color A should be on for. (s) */
			public final Param<Double> timeOnB = new Param<>(this.fx, 0.75, LimelightFX.time);
			/** The time color A should be off for. (s) */
			public final Param<Double> timeOffB = new Param<>(this.fx, 0.25, LimelightFX.time);
			/** The time between color B and color A (if set to repeat). (s) */
			public final Param<Double> timeRepeat = new Param<>(this.fx, 1.0, LimelightFX.time);

			/** How many times color A should blink. */
			public final Param<Integer> blinkCountA = new Param<>(this.fx, 3);
			/** How many times color B should blink. */
			public final Param<Integer> blinkCountB = new Param<>(this.fx, 3);
			/** How many times the entire sequence should repeat. Set to 0 to repeat infinitely. */
			public final Param<Integer> repeatCount = new Param<>(this.fx, 0);

			/** Time to fade in color A. (s) */
			public final Param<Double> fadeInA = new Param<>(this.fx, 0.5, LimelightFX.time);
			/** Time to fade out color A. (s) */
			public final Param<Double> fadeOutA = new Param<>(this.fx, 0.5, LimelightFX.time);
			/** Time to fade in color B. (s) */
			public final Param<Double> fadeInB = new Param<>(this.fx, 0.1, LimelightFX.time);
			/** Time to fade out color B. (s) */
			public final Param<Double> fadeOutB = new Param<>(this.fx, 1.0, LimelightFX.time);
		}

		/** A simple configurable blink pattern. */
		public static class Scroll extends Behavior<Scroll> {
			Scroll(final LimelightFX fx, final Module module, final int layer) {
				super(Kind.Scroll, fx, module, layer);

				this.params = new Param<?>[] {
					this.colorA,
					this.colorB,

					this.speed,
					this.widthA,
					this.widthB, };
			}

			/** The A color. */
			public final Param<Color> colorA = new Param<>(this.fx, new Color(255, 0, 0));
			/** The B color. */
			public final Param<Color> colorB = new Param<>(this.fx, new Color(0, 255, 0));

			/** Speed fraction. (frac) */
			public final Param<Double> speed = new Param<>(this.fx, 1.0, LimelightFX.time);
			/** Width of color A. (px) */
			public final Param<Integer> widthA = new Param<>(this.fx, 2);
			/** Width of color B. (px) */
			public final Param<Integer> widthB = new Param<>(this.fx, 2);
		}

		/** Battlestar Galactica quote not included. */
		public static class Cylon extends Behavior<Cylon> {
			Cylon(final LimelightFX fx, final Module module, final int layer) {
				super(Kind.Cylon, fx, module, layer);

				this.params = new Param<?>[] {
					this.colorA,

					this.speed,
					this.fade, };
			}

			/** The A color. */
			public final Param<Color> colorA = new Param<>(this.fx, new Color(255, 0, 0));
			/** The B color. */
			public final Param<Color> colorB = new Param<>(this.fx, new Color(0, 255, 0));

			/** How fast to sweep. (s, max is {@code LimelightFX.fps}) */
			public final Param<Double> speed = new Param<>(this.fx, 1.0, LimelightFX.time);
			/** Fade percentage over a second (px) */
			public final Param<Double> fade = new Param<>(this.fx, 0.25, LimelightFX.time);
			/** Width of color A. (px) */
			public final Param<Integer> flip = new Param<>(this.fx, 0);
		}

		/** Scrolls some chevrons in a direction. */
		public static class Chevrons extends Behavior<Chevrons> {
			Chevrons(final LimelightFX fx, final Module module, final int layer) {
				super(Kind.Chevrons, fx, module, layer);

				this.params = new Param<?>[] {
					this.colorA,

					this.speed,
					this.widthB, };
			}

			/** The A color. */
			public final Param<Color> colorA = new Param<>(this.fx, new Color(255, 0, 0));
			/** The B color. */
			public final Param<Color> colorB = new Param<>(this.fx, new Color(0, 255, 0));

			/** Width of color A. (px) */
			public final Param<Integer> widthA = new Param<>(this.fx, 0);
			/** Width of color B. (px) */
			public final Param<Integer> widthB = new Param<>(this.fx, 0);
			/** Speed fraction. (frac) */
			public final Param<Double> speed = new Param<>(this.fx, 30.0, LimelightFX.speed);
		}

		public static class Image extends Behavior<Image> {
			Image(final LimelightFX fx, final Module module, final int layer) {
				super(Kind.Image, fx, module, layer);

				this.params = new Param<?>[] { this.filepath, this.xOffset, this.yOffset };
			}

			/**
			 * The image to display. Loaded from the SD card.
			 *
			 * @apiNote Images are loaded from the SD card, use the PGImageTool to put them in a format understood by the FX.
			 */
			public final Param<String> filepath = new Param<>(this.fx, "");

			/** The X offset of the image */
			public final Param<Integer> xOffset = new Param<>(this.fx, 0);

			/** The Y offset of the image */
			public final Param<Integer> yOffset = new Param<>(this.fx, 0);

			public final Image of(final String filepath) {
				this.filepath.set(filepath);
				return this;
			}
		}

		protected Behavior(final Kind kind, final LimelightFX fx, final Module module, final int layer) {
			this.fx = fx;
			this.module = module;
			this.layer = module.layers[layer];

			this.kind = kind;
			this.active = new Param<>(this.fx, false);

			this.fused.add(this);
		}

		public final LimelightFX fx;
		public final Module module;
		public final Layer layer;

		private final Kind kind;
		private final Param<Boolean> active;
		protected volatile Param<?>[] params;
		private volatile HashSet<Behavior<?>> fused = new HashSet<>();

		@SuppressWarnings("unchecked")
		public final T on(final Module module, final int layer) {
			// doesnt need synchronized, background thread doesn't exist yet

			if(this.fx.initialized)
				throw new Error("[ERROR] LimelightFX: Cannot create/modify Behaviors after initialization is complete");

			if(module == null) {
				System.err.println("[WARNING] LimelightFX: Null module used in `.on(module, layer)`");
			}

			T fused;
			try {
				final Constructor<T> constructor = ((Class<T>) this.getClass())
					.getDeclaredConstructor(LimelightFX.class, Module.class, int.class);
				constructor.setAccessible(true);
				fused = constructor.newInstance(this.fx, module, layer);
			} catch(final Exception e) {
				throw new Error(e);
			}

			module.layers[layer].behaviors.add(fused);

			this.fuse(fused);
			fused.link((T) this);
			fused.params = this.params;

			return (T) this;
		}

		@SuppressWarnings({ "unchecked" })
		public final T on(final Module[] modules, final int layer) {
			for(final Module module : modules)
				this.on(module, layer);

			return (T) this;
		}

		@SuppressWarnings({ "unchecked" })
		public final T fuse(final Behavior<?> with) {
			if(this.fx.initialized)
				throw new Error("[ERROR] LimelightFX: Cannot create/modify Behaviors after initialization is complete");

			this.fused.addAll(with.fused);
			with.fused = this.fused;
			with.active.link(this.active);

			final HashSet<Layer> seen = new HashSet<>();

			for(final Behavior<?> fused : this.fused) {
				if(fused.fx != this.fx)
					throw new Error("[ERROR] LimelightFX: Cannot fuse with a Behavior that is on a different FX core!");
				if(fused.layer == null) continue;
				if(seen.contains(fused.layer))
					throw new Error(
						"[ERROR] LimelightFX: Multiple fused Behaviors on the same Module and Layer! This state is invalid, as none of them may activate!"
					);
				seen.add(fused.layer);
			}

			return (T) this;
		}

		public synchronized final void activate() {
			if(!this.active.set(true)) return;

			for(final Behavior<?> fused : this.fused)
				if(fused.layer != null) for(final Behavior<?> beh : fused.layer.behaviors)
					if(beh != fused) beh.active.set(false);
		}

		public synchronized final void deactivate() { this.active.set(false); }

		/**
		 * Creates a command that starts and ends this behavior
		 */
		public final Command command() {
			return new FunctionalCommand(this::activate, () -> {
			}, interrupted -> this.deactivate(), () -> false);
		}

		final String paramstr() {
			String current = "";

			for(final Param<?> param : this.params)
				current += " " + param.toString();

			return current;
		}

		@SuppressWarnings({ "unchecked" })
		public final T link(final T fused) {
			if(this.fx.initialized)
				throw new Error("[ERROR] LimelightFX: Cannot create/modify Behaviors after initialization is complete");
			if(fused.getClass() != this.getClass())
				throw new Error("[ERROR] LimelightFX: Cannot link with a Behavior of a different class!");

			for(int i = 0; i < this.params.length; i++)
				this.params[i] = fused.params[i];

			return (T) this;
		}

		@SuppressWarnings({ "unchecked" })
		public final T cfg(final Consumer<T> configurator) {
			configurator.accept((T) this);

			return (T) this;
		}
	}

	private static final class Selector {
		Selector(final Supplier<Behavior<?>> selector) { this.selector = selector; }

		public final Supplier<Behavior<?>> selector;

		private Behavior<?> last = null;

		final void update() {
			final Behavior<?> beh = this.selector.get();

			if(beh != null) beh.activate();
			else if(this.last != null) this.last.deactivate();

			this.last = beh;
		}
	}

	// IMPLEMENTATION //

	private boolean initialized;

	private volatile Function<String, Boolean> serial;

	private final Module orphan = new Module(-1, this, null, Module.Rotation.R0);
	private final ArrayList<Module> modules = new ArrayList<>();
	private final List<String> rawMessageQueue = Collections.synchronizedList(new ArrayList<>());

	private final ArrayList<Selector> selectors = new ArrayList<>();

	/**
	 * Create a module.
	 *
	 * @param geo
	 *                The {@code Geometry} of the module
	 * @param rot
	 *                The {@code Rotation} of the module
	 * @return The module
	 * @apiNote Cannot be used after {@code LimelightFX.initialize(writer)}
	 */
	public final Module module(final Module.Geometry geo, final Module.Rotation rot) {
		if(this.initialized)
			throw new Error("[ERROR] LimelightFX: Cannot create Modules after initialization is complete");

		final Module module = new Module(this.modules.size() + 1, this, geo, rot);
		for(int i = 0; i < LimelightFX.maxLayers; i++)
			module.layers[i] = new Layer(i);

		this.modules.add(module);

		return module;
	}

	/**
	 * Creates a behavior. Call {@code .on(module, layer)} method to put it on modules
	 *
	 * @param kind
	 *                 The behavior class. (Example: {@code LimelightFX.Behavior.BlinkBehavior.class})
	 * @return The behavior
	 * @see Behavior#on(module, layer)
	 * @apiNote Cannot be used after {@code LimelightFX.initialize(writer)}
	 */
	public final <T extends Behavior<T>> T behavior(final Class<T> kind) {
		if(this.initialized)
			throw new Error("[ERROR] LimelightFX: Cannot create Behaviors after initialization is complete");

		T beh;
		try {
			final Constructor<T> constructor = kind.getDeclaredConstructor(LimelightFX.class, Module.class, int.class);
			constructor.setAccessible(true);
			beh = constructor.newInstance(this, this.orphan, 0);
		} catch(final Exception e) {
			throw new Error("[ERROR] LimelightFX: Behavior creation failed", e);
		}

		return beh;
	}

	/**
	 * Creates a selector. Selectors are a utility that makes behavior selection easier, the passed supplier is invoked in a periodic loop.
	 *
	 * @param selector
	 *                     The selector. If it returns a behavior, that behavior is activated and the previous behavior is deactivated, if it returns null, no new behavior is activated.
	 * @apiNote Cannot be used after {@code LimelightFX.initialize(writer)}
	 */
	public final void selector(final Supplier<Behavior<?>> selector) {
		if(this.initialized)
			throw new Error("[ERROR] LimelightFX: Cannot create Selectors after initialization is complete");

		this.selectors.add(new Selector(selector));
	}

	/**
	 * Initialize the FX. Use this after creating all your modules/behaviors/selectors.
	 *
	 * @param port
	 *                 The serial port that is connected to the LimelightFX Core.
	 */
	public final void initialize(final Supplier<Function<String, Boolean>> writer) {
		if(this.initialized) throw new Error("[ERROR] LimelightFX: Cannot initialize, already initialized!");
		this.initialized = true;

		System.out.println("LimelightFX: Spinning up serial thread...");
		new Thread(() -> this.serial(writer), "LimelightFXSerialThread").start();
	}

	/**
	 * Play a tone on the FX Core.
	 *
	 * @param wave
	 *                     The waveform to use.
	 * @param freq
	 *                     The frequency to play at. (Hz)
	 * @param duration
	 *                     The duration to play for. (s)
	 * @param channel
	 *                     The channel to play on. (0-1)
	 * @param volume
	 *                     The volume to play at. (0-1)
	 */
	public final
		void
		sound(final WaveForm wave, final int freq, final double duration, final int channel, final double volume) {
		synchronized(this.rawMessageQueue) {
			this.rawMessageQueue
				.add(
					String
						.format(
							"tone %d %d %d %d %d",
							freq,
							(int) Math.floor(duration * 1000),
							wave.id,
							channel,
							(int) Math.min(Math.max(Math.floor(volume * 255), 0), 255)
						)
				);
		}
	}

	/**
	 * Play a system sound on the FX Core.
	 *
	 * @param sound
	 *                    The sound to play.
	 * @param loops
	 *                    How many times to play it.
	 * @param channel
	 *                    The channel to play on. (0-1)
	 * @param volume
	 *                    The volume to play at. (0-1)
	 */
	public final void sound(final SystemSound sound, final int loops, final int channel, final double volume) {
		synchronized(this.rawMessageQueue) {
			this.rawMessageQueue
				.add(
					String
						.format(
							"sound %d %d %d %d",
							sound.id,
							channel,
							loops,
							(int) Math.min(Math.max(Math.floor(volume * 255), 0), 255)
						)
				);
		}
	}

	/**
	 * Play a sound from the SD card.
	 *
	 * @param filename
	 *                     The file to play.
	 * @param channel
	 *                     The channel to play on. (0-1)
	 * @param volume
	 *                     The volume to play at. (0-1)
	 */
	public final void sound(final String filename, final int channel, final double volume) {
		synchronized(this.rawMessageQueue) {
			this.rawMessageQueue
				.add(
					String
						.format(
							"stream %s %d %d",
							filename,
							channel,
							(int) Math.min(Math.max(Math.floor(volume * 255), 0), 255)
						)
				);
		}
	}

	@Override
	public void periodic() {
		if(!this.initialized || this.serial == null) return; // exploded

		for(final Selector selector : this.selectors)
			selector.update();
	}

	// SERIAL //

	/** Writes data to the port and disables the FX if write fails */
	private final void write(final String data, final Object... args) {
		try {
			final String command = String.format(data, args);
			if(this.serial == null) return;

			if(!this.serial.apply(command)) this.disable("cannot write");
		} catch(final Exception e) {
			this.disable(e.getMessage());
			e.printStackTrace();
		}
	}

	/** Disable the FX */
	private final void disable(final String reason) {
		if(this.serial == null) return;

		this.serial = null;
		System.err.println("LimelightFX: FX disabled (" + reason + ")");
	}

	/** Serial thread */
	private final void serial(final Supplier<Function<String, Boolean>> writer) {
		System.err.println("LimelightFX: Initializing Serial...");
		this.serial = writer.get();
		System.err.println("LimelightFX: Waiting for connection to finish initializing...");

		try {
			Thread.sleep(1000);
		} catch(final Exception e) {
		}

		System.err.println("LimelightFX: Initializing FX...");
		this.write("init 1");
		for(final Module module : this.modules)
			this.write("module %d %d %d %d", module.geo.kind.id, module.geo.width, module.geo.height, module.rot.id);
		this.write("start");
		System.err.println("LimelightFX: Waiting for LimelightFX to finish initializing...");

		try {
			Thread.sleep(500);
		} catch(final Exception e) {
		}
		System.err.println("LimelightFX: Initialized");

		while(true) {
			if(this.serial == null) return; // exploded

			synchronized(this.rawMessageQueue) {
				for(final String command : this.rawMessageQueue)
					this.write(command);

				this.rawMessageQueue.clear();
			}

			for(final Module module : this.modules)
				for(final Layer layer : module.layers) {
					boolean has_active = false;

					for(final Behavior<?> beh : layer.behaviors) {
						has_active |= beh.active.get();

						if(!beh.active.get()) continue;

						if(beh.active.dirty())
							this.write("beh %d %d %d%s", module.id, layer.id, beh.kind.id, beh.paramstr());
						else
							for(final Behavior.Param<?> param : beh.params)
							if(param.dirty()) this.write("p %d %d %d %s", module.id, layer.id, param.toString());
					}

					if(!has_active && layer.wasActive) this.write("beh %d %d -1", module.id, layer.id);

					layer.wasActive = has_active;
				}

			for(final Module mod : this.modules)
				for(final Layer layer : mod.layers)
					for(final Behavior<?> beh : layer.behaviors) {
						beh.active.clearDirty();
						for(final Behavior.Param<?> param : beh.params)
							param.clearDirty();
					}
		}
	}
}
