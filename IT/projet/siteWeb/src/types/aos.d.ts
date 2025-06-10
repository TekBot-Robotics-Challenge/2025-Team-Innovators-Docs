declare namespace AOS {
  interface AOSSettings {
    // Global settings
    disable?: boolean | string | (() => boolean); // Accepts boolean, 'phone', 'tablet', 'mobile', or function returning boolean
    startEvent?: string; // Name of the event dispatched on the document, that AOS should initialize on
    initClassName?: string; // Class applied after initialization
    animatedClassName?: string; // Class applied on animation
    useClassNames?: boolean; // If true, will add content of `data-aos` as classes on scroll
    disableMutationObserver?: boolean; // Disables automatic mutations' detections
    debounceDelay?: number; // The delay on debounce used while resizing window
    throttleDelay?: number; // The delay on throttle used while scrolling the page

    // Settings that can be overridden on per-element basis, by `data-aos-*` attributes
    offset?: number; // Offset (in px) from the original trigger point
    delay?: number; // Values from 0 to 3000, with step 50ms
    duration?: number; // Values from 0 to 3000, with step 50ms
    easing?: string; // Default easing for AOS animations
    once?: boolean; // Whether animation should happen only once - while scrolling down
    mirror?: boolean; // Whether elements should animate out while scrolling past them
    anchorPlacement?: string; // Defines which position of the element regarding to window should trigger the animation
  }

  interface AOSStatic {
    init(settings?: AOSSettings): void;
    refresh(hard?: boolean): void;
    refreshHard(): void;
  }
}

declare const AOS: AOS.AOSStatic;

declare global {
  interface Window {
    AOS: AOS.AOSStatic;
  }
}

export = AOS;
export as namespace AOS;