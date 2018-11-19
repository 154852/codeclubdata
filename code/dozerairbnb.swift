import Cocoa
import MASShortcut
import Fabric
import Crashlytics
import AXSwift
import Sparkle

private let dozerStatusItem = DozerStatusItem()

@NSApplicationMain
class AppDelegate: NSObject, NSApplicationDelegate {
  
  func applicationDidFinishLaunching(_ notification: Notification) {
    
    UserDefaults.standard.register(defaults: ["NSApplicationCrashOnExceptions" : true])
    Fabric.with([Crashlytics.self])
   
    firstRun()
    
  }
  
  func continueAppLife() {
    dozerStatusItem.show()
    
    // Menu Bar Hover
    NSEvent.addLocalMonitorForEvents(matching: .mouseMoved) { (event) -> NSEvent? in
      if !UserDefaults.standard.bool(forKey: UserDefaultKeys.ShowIconsOnHover.defaultKey) {
        return event
      }
      let mouseLocation = NSEvent.mouseLocation
      dozerStatusItem.handleMouseMoved(mouseLocation: mouseLocation)
      return event
    }
    
    NSEvent.addGlobalMonitorForEvents(matching: .mouseMoved) { (_) in
      if !UserDefaults.standard.bool(forKey: UserDefaultKeys.ShowIconsOnHover.defaultKey) {
        return
      }
      let mouseLocation = NSEvent.mouseLocation
      dozerStatusItem.handleMouseMoved(mouseLocation: mouseLocation)
    }
    
    // Menu Bar Click
    NSEvent.addLocalMonitorForEvents(matching: .leftMouseUp) { (event) -> NSEvent? in
      if UserDefaults.standard.bool(forKey: UserDefaultKeys.ShowIconsOnHover.defaultKey) {
        return event
      }
      let mouseLocation = NSEvent.mouseLocation
      dozerStatusItem.handleClickOnMenuBar(mouseLocation: mouseLocation)
      return event
    }
    
    NSEvent.addGlobalMonitorForEvents(matching: .leftMouseUp) { (_) in
      if UserDefaults.standard.bool(forKey: UserDefaultKeys.ShowIconsOnHover.defaultKey) {
        return
      }
      let mouseLocation = NSEvent.mouseLocation
      dozerStatusItem.handleClickOnMenuBar(mouseLocation: mouseLocation)
    }
    
    // bind global shortcut
    MASShortcutBinder.shared()?.bindShortcut(
      withDefaultsKey: UserDefaultKeys.Shortcuts.ToggleMenuItems,
      toAction: {
        dozerStatusItem.toggle()
    })
    
    SUUpdater.shared()?.automaticallyChecksForUpdates = true
  }

  @objc func showPreferences() {
    PreferencesController.shared.showPreferencesPane()
  }
  
}

import Cocoa

struct UserDefaultKeys {
  private init(){}
  struct Shortcuts {
    private init(){}
    #if !DEBUG
      static let ToggleMenuItems = "toggleMenuItems"
    #else
      static let ToggleMenuItems = "toggleMenuItemsDEBUG"
    #endif
  }
  struct FirstRun {
    private init(){}
    #if !DEBUG
      static let defaultKey = "firstRunV2.1.0"
    #else
      static let defaultKey = "firstRunV2.1.0DEBUG"
    #endif
  }
  struct ShowIconsOnHover {
    private init(){}
    #if !DEBUG
    static let defaultKey = "showIconsOnHover"
    #else
    static let defaultKey = "showIconsOnHoverDEBUG"
    #endif
  }
}

struct StoryboardNames {
  private init(){}
    static let Preferences = NSStoryboard.Name("Preferences")
}

import Cocoa
import AXSwift

class DozerStatusItem {
  
  let statusItem = NSStatusBar.system.statusItem(withLength: NSStatusItem.variableLength)
  
  let shownLength:CGFloat = 20
  let hiddenLength:CGFloat = 10000
  
  init() {
    statusItem.length = shownLength
    
    guard let mainStatusItemButton = statusItem.button else {
      fatalError("main status item button failed")
    }
    
    mainStatusItemButton.target = self
    mainStatusItemButton.action = #selector(statusItemClicked(sender:))
    mainStatusItemButton.image = Icons().statusBarIcon
    mainStatusItemButton.image!.isTemplate = true
    mainStatusItemButton.sendAction(on: [.leftMouseDown, .rightMouseDown])
  }
  
  deinit {
    print("status item has been deallocated")
  }
  
  func show() {
    statusItem.length = shownLength
  }
  
  func hide() {
    statusItem.length = hiddenLength
  }
  
  func toggle() {
    if isShown {
      hide()
    } else {
      show()
    }
  }
  
  @objc func statusItemClicked(sender: AnyObject?) {
    guard let currentEvent = NSApp.currentEvent else {
      NSLog("read current event failed")
      return
    }
    
    if currentEvent.type == NSEvent.EventType.leftMouseDown {
      handleLeftClick()
    }
    
    if currentEvent.type == NSEvent.EventType.rightMouseDown {
      handleRightClick()
    }
  }
  
  internal func handleLeftClick() {
    self.hide()
  }
  
  internal func handleRightClick() {
    PreferencesController.shared.showPreferencesPane()
  }
  
  var isShown:Bool {
    return (statusItem.length == shownLength)
  }
  
  var isHidden:Bool {
    return (statusItem.length == hiddenLength)
  }
  
  internal func handleMouseMoved(mouseLocation:NSPoint) {
    if isMouseInStatusBar(with: mouseLocation) && ListenForMouseExit.shared.mouseHasExited {
      self.show()
      ListenForMouseExit.shared.mouseHasExited = false
    } else if !isMouseInStatusBar(with: mouseLocation) {
      ListenForMouseExit.shared.mouseHasExited = true
    }
  }
  
  internal func handleClickOnMenuBar(mouseLocation:NSPoint) {
    if isMouseInStatusBarSaveSpace(with: mouseLocation) {
      self.show()
    }
  }
  
  func isMouseInStatusBarSaveSpace(with mouseLocation:NSPoint) -> Bool {
    
    let statusBarHeight = NSStatusBar.system.thickness
    guard let menuBarOwningApp = NSWorkspace.shared.menuBarOwningApplication else {
      fatalError()
    }
    
    for screen in NSScreen.screens {
      var frame = screen.frame
      frame.origin.y = frame.origin.y + frame.height - statusBarHeight - 2
      frame.size.height = statusBarHeight + 3
      frame.origin.x = getLastMenuItemXPosition(from: menuBarOwningApp)
      frame.size.width = xPositionOnScreen - frame.origin.x + shownLength
      if frame.contains(mouseLocation) {
        return true
      }
    }
    
    return false
  }
  
  internal var xPositionOnScreen:CGFloat {
    guard let dozerIconFrame = statusItem.button?.window?.frame else {
      return 0
    }
    let dozerIconXPosition = dozerIconFrame.origin.x + dozerIconFrame.width - shownLength
    return dozerIconXPosition
  }
  
}

import Cocoa
import AVKit
import AXSwift

class BoardingScreen: NSViewController {

  @IBOutlet var ContinueButton: NSButton!
  @IBOutlet var moveDozer: AVPlayerView!
  @IBOutlet var EnableDozerLabel: NSTextField!
  @IBOutlet var OpenAccessibilityButton: NSButton!
  
  override func viewDidLoad() {
    super.viewDidLoad()
    self.preferredContentSize = NSSize(width: self.view.frame.size.width, height: self.view.frame.size.height)
    
    // button config
    ContinueButton.focusRingType = .none
    
    if UIElement.isProcessTrusted(withPrompt: false) {
      EnableDozerLabel.isHidden = true
    }
    
    OpenAccessibilityButton.isEnabled = !UIElement.isProcessTrusted(withPrompt: false)
    
    let videoURL = Bundle.main.url(forResource: "Demo", withExtension: "mp4")!
    let player = AVPlayer(url: videoURL)
    moveDozer.player = player
    moveDozer.controlsStyle = .none
    moveDozer.player?.actionAtItemEnd = .none
    
    NotificationCenter.default.addObserver(
      forName: .AVPlayerItemDidPlayToEndTime,
      object: moveDozer.player?.currentItem,
      queue: .main) { _ in
        self.moveDozer.player?.seek(to: CMTime.zero)
        self.moveDozer.player?.play()
    }
  }
  
  override func viewDidAppear() {
    super.viewDidAppear()
    moveDozer.player?.play()
  }

  func show() {
    let window = NSWindow(
      contentRect: NSRect(x: 0, y: 0,
      width: self.view.frame.width,
      height: self.view.frame.height),
      styleMask: [.titled, .miniaturizable],
      backing: .buffered,
      defer: false)
    window.contentView?.addSubview(self.view)
    window.makeKeyAndOrderFront(nil)
    NSApp.activate(ignoringOtherApps: true)
    self.centerOnScreen()
  }
  
  @IBAction func ContinueButtonPressed(_ sender: NSButton) {
    if UIElement.isProcessTrusted(withPrompt: true) {
      moveDozer.player?.pause()
      moveDozer.player = nil
      let window = self.view.window
      window?.orderOut(nil)
      AppDelegate().continueAppLife()
      
      UserDefaults.standard.set(true, forKey: UserDefaultKeys.FirstRun.defaultKey)
    }
  }
  
  @IBAction func OpenAccessibility(_ sender: NSButton) {
    _ = UIElement.isProcessTrusted(withPrompt: true)
  }
  
}

extension BoardingScreen {
  
  // Warning: safely unwrap parent window
  func centerOnScreen() {
    let window = self.view.window!
    let centerY = window.screen!.frame.height / 2 - self.view.frame.height / 2
    let centerX = window.screen!.frame.width / 2 - self.view.frame.width / 2
    window.setFrameOrigin(NSPoint(x: centerX, y: centerY))
  }
  
}

import Cocoa

final class PreferencesController {
  private init() {}
  
  required internal init?(coder: NSCoder) {
    fatalError("init(coder:) has not been implemented")
  }
  
  static let shared = PreferencesController()
  internal var preferencesController:NSWindowController?
  
  @objc func showPreferencesPane() {
    if preferencesController == nil {
      let storyboard = NSStoryboard(name: StoryboardNames.Preferences, bundle: nil)
      preferencesController = storyboard.instantiateInitialController() as? NSWindowController
      preferencesController?.window?.level = .statusBar
    }
    
    if preferencesController != nil {
      preferencesController!.showWindow(nil)
      NSApp.activate(ignoringOtherApps: true)
    }
  }
  
}

import UIKit

/**
 * A simple class for laying out a collection of views with a convenient API, while leveraging the
 * power of Auto Layout.
 */
open class AloeStackView: UIScrollView {

  // MARK: Lifecycle
  public init() {
    super.init(frame: .zero)
    setUpViews()
    setUpConstraints()
  }

  required public init?(coder aDecoder: NSCoder) {
    fatalError("init(coder:) has not been implemented")
  }

  // MARK: - Public
  // MARK: Adding and Removing Rows
  /// Adds a row to the end of the stack view.
  ///
  /// If `animated` is `true`, the insertion is animated.
  open func addRow(_ row: UIView, animated: Bool = false) {
    insertCell(withContentView: row, atIndex: stackView.arrangedSubviews.count, animated: animated)
  }

  /// Adds multiple rows to the end of the stack view.
  ///
  /// If `animated` is `true`, the insertions are animated.
  open func addRows(_ rows: [UIView], animated: Bool = false) {
    rows.forEach { addRow($0, animated: animated) }
  }

  /// Adds a row to the beginning of the stack view.
  ///
  /// If `animated` is `true`, the insertion is animated.
  open func prependRow(_ row: UIView, animated: Bool = false) {
    insertCell(withContentView: row, atIndex: 0, animated: animated)
  }

  /// Adds multiple rows to the beginning of the stack view.
  ///
  /// If `animated` is `true`, the insertions are animated.
  open func prependRows(_ rows: [UIView], animated: Bool = false) {
    rows.reversed().forEach { prependRow($0, animated: animated) }
  }

  /// Inserts a row above the specified row in the stack view.
  ///
  /// If `animated` is `true`, the insertion is animated.
  open func insertRow(_ row: UIView, before beforeRow: UIView, animated: Bool = false) {
    guard
      let cell = beforeRow.superview as? StackViewCell,
      let index = stackView.arrangedSubviews.index(of: cell) else { return }

    insertCell(withContentView: row, atIndex: index, animated: animated)
  }

  /// Inserts multiple rows above the specified row in the stack view.
  ///
  /// If `animated` is `true`, the insertions are animated.
  open func insertRows(_ rows: [UIView], before beforeRow: UIView, animated: Bool = false) {
    rows.forEach { insertRow($0, before: beforeRow, animated: animated) }
  }

  /// Inserts a row below the specified row in the stack view.
  ///
  /// If `animated` is `true`, the insertion is animated.
  open func insertRow(_ row: UIView, after afterRow: UIView, animated: Bool = false) {
    guard
      let cell = afterRow.superview as? StackViewCell,
      let index = stackView.arrangedSubviews.index(of: cell) else { return }

    insertCell(withContentView: row, atIndex: index + 1, animated: animated)
  }

  /// Inserts multiple rows below the specified row in the stack view.
  ///
  /// If `animated` is `true`, the insertions are animated.
  open func insertRows(_ rows: [UIView], after afterRow: UIView, animated: Bool = false) {
    _ = rows.reduce(afterRow) { currentAfterRow, row in
      insertRow(row, after: currentAfterRow, animated: animated)
      return row
    }
  }

  /// Removes the given row from the stack view.
  ///
  /// If `animated` is `true`, the removal is animated.
  open func removeRow(_ row: UIView, animated: Bool = false) {
    if let cell = row.superview as? StackViewCell {
      removeCell(cell, animated: animated)
    }
  }

  /// Removes the given rows from the stack view.
  ///
  /// If `animated` is `true`, the removals are animated.
  open func removeRows(_ rows: [UIView], animated: Bool = false) {
    rows.forEach { removeRow($0, animated: animated) }
  }

  /// Removes all the rows in the stack view.
  ///
  /// If `animated` is `true`, the removals are animated.
  open func removeAllRows(animated: Bool = false) {
    stackView.arrangedSubviews.forEach { view in
      if let cell = view as? StackViewCell {
        removeRow(cell.contentView, animated: animated)
      }
    }
  }

  // MARK: Accessing Rows
  /// Returns an array containing of all the rows in the stack view.
  ///
  /// The rows in the returned array are in the order they appear visually in the stack view.
  open func getAllRows() -> [UIView] {
    var rows: [UIView] = []
    stackView.arrangedSubviews.forEach { cell in
      if let cell = cell as? StackViewCell {
        rows.append(cell.contentView)
      }
    }
    return rows
  }

  /// Returns `true` if the given row is present in the stack view, `false` otherwise.
  open func containsRow(_ row: UIView) -> Bool {
    guard let cell = row.superview as? StackViewCell else { return false }
    return stackView.arrangedSubviews.contains(cell)
  }

  // MARK: Hiding and Showing Rows
  /// Hides the given row, making it invisible.
  ///
  /// If `animated` is `true`, the change is animated.
  open func hideRow(_ row: UIView, animated: Bool = false) {
    setRowHidden(row, isHidden: true, animated: animated)
  }

  /// Hides the given rows, making them invisible.
  ///
  /// If `animated` is `true`, the changes are animated.
  open func hideRows(_ rows: [UIView], animated: Bool = false) {
    rows.forEach { hideRow($0, animated: animated) }
  }

  /// Shows the given row, making it visible.
  ///
  /// If `animated` is `true`, the change is animated.
  open func showRow(_ row: UIView, animated: Bool = false) {
    setRowHidden(row, isHidden: false, animated: animated)
  }

  /// Shows the given rows, making them visible.
  ///
  /// If `animated` is `true`, the changes are animated.
  open func showRows(_ rows: [UIView], animated: Bool = false) {
    rows.forEach { showRow($0, animated: animated) }
  }

  /// Hides the given row if `isHidden` is `true`, or shows the given row if `isHidden` is `false`.
  ///
  /// If `animated` is `true`, the change is animated.
  open func setRowHidden(_ row: UIView, isHidden: Bool, animated: Bool = false) {
    guard let cell = row.superview as? StackViewCell else { return }

    if animated {
      UIView.animate(withDuration: 0.3) {
        cell.isHidden = isHidden
        cell.layoutIfNeeded()
      }
    } else {
      cell.isHidden = isHidden
    }
  }

  /// Hides the given rows if `isHidden` is `true`, or shows the given rows if `isHidden` is
  /// `false`.
  ///
  /// If `animated` is `true`, the change are animated.
  open func setRowsHidden(_ rows: [UIView], isHidden: Bool, animated: Bool = false) {
    rows.forEach { setRowHidden($0, isHidden: isHidden, animated: animated) }
  }

  /// Returns `true` if the given row is hidden, `false` otherwise.
  open func isRowHidden(_ row: UIView) -> Bool {
    return (row.superview as? StackViewCell)?.isHidden ?? false
  }

  // MARK: Handling User Interaction
  /// Sets a closure that will be called when the given row in the stack view is tapped by the user.
  ///
  /// The handler will be passed the row.
  open func setTapHandler<RowView: UIView>(forRow row: RowView, handler: ((RowView) -> Void)?) {
    guard let cell = row.superview as? StackViewCell else { return }

    if let handler = handler {
      cell.tapHandler = { contentView in
        guard let contentView = contentView as? RowView else { return }
        handler(contentView)
      }
    } else {
      cell.tapHandler = nil
    }
  }

  // MARK: Styling Rows
  /// The background color of rows in the stack view.
  ///
  /// This background color will be used for any new row that is added to the stack view.
  /// The default color is clear.
  open var rowBackgroundColor = UIColor.clear
    
  /// The highlight background color of rows in the stack view.
  ///
  /// This highlight background color will be used for any new row that is added to the stack view.
  /// The default color is #D9D9D9 (RGB 217, 217, 217).
  open var rowHighlightColor = AloeStackView.defaultRowHighlightColor

  /// Sets the background color for the given row to the `UIColor` provided.
  open func setBackgroundColor(forRow row: UIView, color: UIColor) {
    (row.superview as? StackViewCell)?.rowBackgroundColor = color
  }

  /// Sets the background color for the given rows to the `UIColor` provided.
  open func setBackgroundColor(forRows rows: [UIView], color: UIColor) {
    rows.forEach { setBackgroundColor(forRow: $0, color: color) }
  }

  /// Specifies the default inset of rows.
  ///
  /// This inset will be used for any new row that is added to the stack view.
  ///
  /// You can use this property to add space between a row and the left and right edges of the stack
  /// view and the rows above and below it. Positive inset values move the row inward and away
  /// from the stack view edges and away from rows above and below.
  ///
  /// The default inset is 15pt on each side and 12pt on the top and bottom.
  open var rowInset = UIEdgeInsets(
    top: 12,
    left: AloeStackView.defaultSeparatorInset.left,
    bottom: 12,
    // Intentional, to match the default spacing of UITableView's cell separators but balanced on
    // each side.
    right: AloeStackView.defaultSeparatorInset.left)

  /// Sets the inset for the given row to the `UIEdgeInsets` provided.
  open func setInset(forRow row: UIView, inset: UIEdgeInsets) {
    (row.superview as? StackViewCell)?.rowInset = inset
  }

  /// Sets the inset for the given rows to the `UIEdgeInsets` provided.
  open func setInset(forRows rows: [UIView], inset: UIEdgeInsets) {
    rows.forEach { setInset(forRow: $0, inset: inset) }
  }

  // MARK: Styling Separators
  /// The color of separators in the stack view.
  ///
  /// The default color matches the default color of separators in `UITableView`.
  open var separatorColor = AloeStackView.defaultSeparatorColor {
    didSet {
      for cell in stackView.arrangedSubviews {
        (cell as? StackViewCell)?.separatorColor = separatorColor
      }
    }
  }

  /// The height of separators in the stack view.
  ///
  /// The default height is 1px.
  open var separatorHeight: CGFloat = 1 / UIScreen.main.scale {
    didSet {
      for cell in stackView.arrangedSubviews {
        (cell as? StackViewCell)?.separatorHeight = separatorHeight
      }
    }
  }

  /// Specifies the default inset of row separators.
  ///
  /// Only left and right insets are honored. This inset will be used for any new row that is added
  /// to the stack view. The default inset matches the default inset of cell separators in
  /// `UITableView`, which are 15pt on the left and 0pt on the right.
  open var separatorInset: UIEdgeInsets = AloeStackView.defaultSeparatorInset

  /// Sets the separator inset for the given row to the `UIEdgeInsets` provided.
  ///
  /// Only left and right insets are honored.
  open func setSeperatorInset(forRow row: UIView, inset: UIEdgeInsets) {
    (row.superview as? StackViewCell)?.separatorInset = inset
  }

  /// Sets the separator inset for the given rows to the `UIEdgeInsets` provided.
  ///
  /// Only left and right insets are honored.
  open func setSeperatorInset(forRows rows: [UIView], inset: UIEdgeInsets) {
    rows.forEach { setSeperatorInset(forRow: $0, inset: inset) }
  }

  // MARK: Hiding and Showing Separators
  /// Specifies the default visibility of row separators.
  ///
  /// When `true`, separators will be hidden for any new rows added to the stack view.
  /// When `false, separators will be visible for any new rows added. Default is `false`, meaning
  /// separators are visible for any new rows that are added.
  open var hidesSeparatorsByDefault = false

  /// Hides the separator for the given row.
  open func hideSeparator(forRow row: UIView) {
    if let cell = row.superview as? StackViewCell {
      cell.shouldHideSeparator = true
      updateSeparatorVisibility(forCell: cell)
    }
  }

  /// Hides separators for the given rows.
  open func hideSeparators(forRows rows: [UIView]) {
    rows.forEach { hideSeparator(forRow: $0) }
  }

  /// Shows the separator for the given row.
  open func showSeparator(forRow row: UIView) {
    if let cell = row.superview as? StackViewCell {
      cell.shouldHideSeparator = false
      updateSeparatorVisibility(forCell: cell)
    }
  }

  /// Shows separators for the given rows.
  open func showSeparators(forRows rows: [UIView]) {
    rows.forEach { showSeparator(forRow: $0) }
  }

  /// Automatically hides the separator of the last cell in the stack view.
  ///
  /// Default is `false`.
  open var automaticallyHidesLastSeparator = false {
    didSet {
      if let cell = stackView.arrangedSubviews.last as? StackViewCell {
        updateSeparatorVisibility(forCell: cell)
      }
    }
  }

  // MARK: Modifying the Scroll Position
  /// Scrolls the given row onto screen so that it is fully visible.
  ///
  /// If `animated` is `true`, the scroll is animated. If the row is already fully visible, this
  /// method does nothing.
  open func scrollRowToVisible(_ row: UIView, animated: Bool = true) {
    guard let superview = row.superview else { return }
    scrollRectToVisible(convert(row.frame, from: superview), animated: animated)
  }

  // MARK: Extending AloeStackView
  /// Returns the `StackViewCell` to be used for the given row.
  ///
  /// An instance of `StackViewCell` wraps every row in the stack view.
  ///
  /// Subclasses can override this method to return a custom `StackViewCell` subclass, for example
  /// to add custom behavior or functionality that is not provided by default.
  ///
  /// If you customize the values of some properties of `StackViewCell` in this method, these values
  /// may be overwritten by default values after the cell is returned. To customize the values of
  /// properties of the cell, override `configureCell(_:)` and perform the customization there,
  /// rather than on the cell returned from this method.
  open func cellForRow(_ row: UIView) -> StackViewCell {
    return StackViewCell(contentView: row)
  }

  /// Allows subclasses to configure the properties of the given `StackViewCell`.
  ///
  /// This method is called for newly created cells after the default values of any properties of
  /// the cell have been set by the superclass.
  ///
  /// The default implementation of this method does nothing.
  open func configureCell(_ cell: StackViewCell) { }

  // MARK: - Private
  private let stackView = UIStackView()

  private func setUpViews() {
    setUpSelf()
    setUpStackView()
  }

  private func setUpSelf() {
    backgroundColor = UIColor.white
  }

  private func setUpStackView() {
    stackView.translatesAutoresizingMaskIntoConstraints = false
    stackView.axis = .vertical
    addSubview(stackView)
  }

  private func setUpConstraints() {
    setUpStackViewConstraints()
  }

  private func setUpStackViewConstraints() {
    NSLayoutConstraint.activate([
      stackView.topAnchor.constraint(equalTo: topAnchor),
      stackView.bottomAnchor.constraint(equalTo: bottomAnchor),
      stackView.leadingAnchor.constraint(equalTo: leadingAnchor),
      stackView.trailingAnchor.constraint(equalTo: trailingAnchor),
      stackView.widthAnchor.constraint(equalTo: widthAnchor)
    ])
  }

  private func createCell(withContentView contentView: UIView) -> StackViewCell {
    let cell = cellForRow(contentView)

    cell.rowBackgroundColor = rowBackgroundColor
    cell.rowHighlightColor = rowHighlightColor
    cell.rowInset = rowInset
    cell.separatorColor = separatorColor
    cell.separatorHeight = separatorHeight
    cell.separatorInset = separatorInset
    cell.shouldHideSeparator = hidesSeparatorsByDefault

    configureCell(cell)

    return cell
  }

  private func insertCell(withContentView contentView: UIView, atIndex index: Int, animated: Bool) {
    let cellToRemove = containsRow(contentView) ? contentView.superview : nil

    let cell = createCell(withContentView: contentView)
    stackView.insertArrangedSubview(cell, at: index)

    if let cellToRemove = cellToRemove as? StackViewCell {
      removeCell(cellToRemove, animated: false)
    }

    updateSeparatorVisibility(forCell: cell)

    // A cell can affect the visibility of the cell before it, e.g. if
    // `automaticallyHidesLastSeparator` is true and a new cell is added as the last cell, so update
    // the previous cell's separator visibility as well.
    if let cellAbove = cellAbove(cell: cell) {
      updateSeparatorVisibility(forCell: cellAbove)
    }

    if animated {
      cell.alpha = 0
      layoutIfNeeded()
      UIView.animate(withDuration: 0.3) {
        cell.alpha = 1
      }
    }
  }

  private func removeCell(_ cell: StackViewCell, animated: Bool) {
    let aboveCell = cellAbove(cell: cell)

    let completion: (Bool) -> Void = { [weak self] _ in
      guard let `self` = self else { return }
      cell.removeFromSuperview()

      // When removing a cell, the cell before the removed cell is the only cell whose separator
      // visibility could be affected, so we need to update its visibility.
      if let aboveCell = aboveCell {
        self.updateSeparatorVisibility(forCell: aboveCell)
      }
    }

    if animated {
      UIView.animate(
        withDuration: 0.3,
        animations: {
          cell.isHidden = true
        },
        completion: completion)
    } else {
      completion(true)
    }
  }

  private func updateSeparatorVisibility(forCell cell: StackViewCell) {
    let isLastCellAndHidingIsEnabled = automaticallyHidesLastSeparator &&
      cell === stackView.arrangedSubviews.last
    let cellConformsToSeparatorHiding = cell.contentView is SeparatorHiding

    cell.isSeparatorHidden =
      isLastCellAndHidingIsEnabled ||
      cellConformsToSeparatorHiding ||
      cell.shouldHideSeparator
  }

  private func cellAbove(cell: StackViewCell) -> StackViewCell? {
    guard let index = stackView.arrangedSubviews.index(of: cell), index > 0 else { return nil }
    return stackView.arrangedSubviews[index - 1] as? StackViewCell
  }

  private static let defaultRowHighlightColor: UIColor = UIColor(red: 217 / 255, green: 217 / 255, blue: 217 / 255, alpha: 1)
  private static let defaultSeparatorColor: UIColor = UITableView().separatorColor ?? .clear
  private static let defaultSeparatorInset: UIEdgeInsets = UITableView().separatorInset

}
import UIKit

internal final class SeparatorView: UIView {

  // MARK: Lifecycle
  internal init() {
    super.init(frame: .zero)
    translatesAutoresizingMaskIntoConstraints = false
  }

  internal required init?(coder aDecoder: NSCoder) {
    fatalError("init(coder:) has not been implemented")
  }

  // MARK: Internal
  internal override var intrinsicContentSize: CGSize {
    #if swift(>=4.2)
    return CGSize(width: UIView.noIntrinsicMetric, height: height)
    #else
    return CGSize(width: UIViewNoIntrinsicMetric, height: height)
    #endif
  }

  internal var color: UIColor {
    get { return backgroundColor ?? .clear }
    set { backgroundColor = newValue }
  }

  internal var height: CGFloat = 1 {
    didSet { invalidateIntrinsicContentSize() }
  }

}

import UIKit

/**
 * A view that wraps every row in a stack view.
 */
open class StackViewCell: UIView {

  // MARK: Lifecycle
  public init(contentView: UIView) {
    self.contentView = contentView

    super.init(frame: .zero)
    translatesAutoresizingMaskIntoConstraints = false
    if #available(iOS 11.0, *) {
      insetsLayoutMarginsFromSafeArea = false
    }

    setUpViews()
    setUpConstraints()
    setUpTapGestureRecognizer()
  }

  public required init?(coder aDecoder: NSCoder) {
    fatalError("init(coder:) has not been implemented")
  }

  // MARK: Open
    
  open override var isHidden: Bool {
    didSet {
      guard isHidden != oldValue else { return }
      separatorView.alpha = isHidden ? 0 : 1
    }
  }

  open var rowHighlightColor = UIColor(red: 217 / 255, green: 217 / 255, blue: 217 / 255, alpha: 1)

  open var rowBackgroundColor = UIColor.clear {
    didSet {
      backgroundColor = rowBackgroundColor
    }
  }

  open var rowInset: UIEdgeInsets {
    get { return layoutMargins }
    set { layoutMargins = newValue }
  }

  open var separatorColor: UIColor {
    get { return separatorView.color }
    set { separatorView.color = newValue }
  }

  open var separatorHeight: CGFloat {
    get { return separatorView.height }
    set { separatorView.height = newValue }
  }

  open var separatorInset: UIEdgeInsets = .zero {
    didSet { updateSeparatorInset() }
  }

  open var isSeparatorHidden: Bool {
    get { return separatorView.isHidden }
    set { separatorView.isHidden = newValue }
  }

  // MARK: Public
  public let contentView: UIView

  // MARK: UIResponder
  open override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
    super.touchesBegan(touches, with: event)
    guard contentView.isUserInteractionEnabled else { return }

    if let contentView = contentView as? Highlightable, contentView.isHighlightable {
      contentView.setIsHighlighted(true)
    }
  }

  open override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
    super.touchesMoved(touches, with: event)
    guard contentView.isUserInteractionEnabled else { return }

    if let contentView = contentView as? Highlightable, contentView.isHighlightable {
      contentView.setIsHighlighted(false)
    }
  }

  open override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
    super.touchesEnded(touches, with: event)
    guard contentView.isUserInteractionEnabled else { return }

    if let contentView = contentView as? Highlightable, contentView.isHighlightable {
      contentView.setIsHighlighted(false)
    }
  }

  // MARK: Internal
  internal var tapHandler: ((UIView) -> Void)? {
    didSet { updateTapGestureRecognizerEnabled() }
  }

  // Whether the separator should be hidden or not for this cell. Note that this doesn't always
  // reflect whether the separator is hidden or not, since, for example, the separator could be
  // hidden because it's the last row in the stack view and
  // `automaticallyHidesLastSeparator` is `true`.
  internal var shouldHideSeparator = false

  // MARK: Private
  private let separatorView = SeparatorView()
  private let tapGestureRecognizer = UITapGestureRecognizer()

  private var separatorLeadingConstraint: NSLayoutConstraint?
  private var separatorTrailingConstraint: NSLayoutConstraint?

  private func setUpViews() {
    setUpSelf()
    setUpContentView()
    setUpSeparatorView()
  }

  private func setUpSelf() {
    clipsToBounds = true
  }

  private func setUpContentView() {
    contentView.translatesAutoresizingMaskIntoConstraints = false
    addSubview(contentView)
  }

  private func setUpSeparatorView() {
    addSubview(separatorView)
  }

  private func setUpConstraints() {
    setUpContentViewConstraints()
    setUpSeparatorViewConstraints()
  }

  private func setUpContentViewConstraints() {
    let bottomConstraint = contentView.bottomAnchor.constraint(equalTo: layoutMarginsGuide.bottomAnchor)
    bottomConstraint.priority = UILayoutPriority(rawValue: UILayoutPriority.required.rawValue - 1)

    NSLayoutConstraint.activate([
      contentView.leadingAnchor.constraint(equalTo: layoutMarginsGuide.leadingAnchor),
      contentView.trailingAnchor.constraint(equalTo: layoutMarginsGuide.trailingAnchor),
      contentView.topAnchor.constraint(equalTo: layoutMarginsGuide.topAnchor),
      bottomConstraint
    ])
  }

  private func setUpSeparatorViewConstraints() {
    let leadingConstraint = separatorView.leadingAnchor.constraint(equalTo: leadingAnchor)
    let trailingConstraint = separatorView.trailingAnchor.constraint(equalTo: trailingAnchor)

    NSLayoutConstraint.activate([
      separatorView.bottomAnchor.constraint(equalTo: bottomAnchor),
      leadingConstraint,
      trailingConstraint
    ])

    separatorLeadingConstraint = leadingConstraint
    separatorTrailingConstraint = trailingConstraint
  }

  private func setUpTapGestureRecognizer() {
    tapGestureRecognizer.addTarget(self, action: #selector(handleTap(_:)))
    addGestureRecognizer(tapGestureRecognizer)
    updateTapGestureRecognizerEnabled()
  }

  @objc private func handleTap(_ tapGestureRecognizer: UITapGestureRecognizer) {
    guard contentView.isUserInteractionEnabled else { return }
    (contentView as? Tappable)?.didTapView()
    tapHandler?(contentView)
  }

  private func updateTapGestureRecognizerEnabled() {
    tapGestureRecognizer.isEnabled = contentView is Tappable || tapHandler != nil
  }

  private func updateSeparatorInset() {
    separatorLeadingConstraint?.constant = separatorInset.left
    separatorTrailingConstraint?.constant = -separatorInset.right
  }

}
import AppKit
import Foundation

// This NSTextField subclass draws strings with a .baselineOffset attribute correctly.
//
// OSX 10.14 fixes a layout issue when using the .baselineOffset attribute, but we use
// this subclass in order to support 10.12 and 10.13. This isn't a general-purpose replacement
// for NSTextField; it should only be used for non-editable labels. Whenever we drop support
// for pre-10.14 we can remove this and use NSTextField directly instead.
public class LNATextField: NSTextField {
  override open class var cellClass: AnyClass? {
    get { return LNATextFieldCell.self }
    set {}
  }

  // Determine the baseline offset from the attributed string value. We store it as a member variable,
  // then we remove it from the attributed string.
  override public var attributedStringValue: NSAttributedString {
    get { return super.attributedStringValue }
    set {
      baselineOffset = newValue.baselineOffset

      let string = NSMutableAttributedString(attributedString: newValue)
      string.removeAttribute(.baselineOffset, range: NSRange(location: 0, length: string.length))
      super.attributedStringValue = string
    }
  }

  fileprivate var baselineOffset: CGFloat?
}

private class LNATextFieldCell: NSTextFieldCell {
  override func drawInterior(withFrame cellFrame: NSRect, in controlView: NSView) {
    if let textView = controlView as? LNATextField,
      let baselineOffset = textView.baselineOffset,
      let lineHeight = attributedStringValue.lineHeight {

      var rect = cellFrame.insetBy(dx: 2, dy: 0)

      rect.origin.y -= baselineOffset

      let truncatesLastVisibleLine = textView.maximumNumberOfLines > 0 &&
        cellFrame.height / lineHeight >= CGFloat(textView.maximumNumberOfLines)

      let options: NSString.DrawingOptions = truncatesLastVisibleLine
        ? [.usesLineFragmentOrigin, .truncatesLastVisibleLine]
        : [.usesLineFragmentOrigin]

      attributedStringValue.draw(with: rect, options: options)
    } else {
      super.drawInterior(withFrame: cellFrame, in: controlView)
    }
  }
}

private extension NSAttributedString {
  var lineHeight: CGFloat? {
    guard
      length > 0,
      let paragraphStyle = attribute(.paragraphStyle, at: 0, effectiveRange: nil) as? NSParagraphStyle
    else { return nil }

    let lineHeight = paragraphStyle.minimumLineHeight

    if lineHeight <= 0 { return nil }

    return lineHeight
  }

  var baselineOffset: CGFloat? {
    guard
      length > 0,
      let lineHeight = lineHeight,
      let font = attribute(.font, at: 0, effectiveRange: nil) as? NSFont
    else { return nil }

    return (lineHeight - font.ascender + font.descender) / 2
  }
}
import Foundation
import AppKit

public class TextStyle {
  public let family: String?
  public let name: String?
  public let weight: NSFont.Weight
  public let size: CGFloat
  public let lineHeight: CGFloat?
  public let kerning: Double
  public let color: NSColor?
  public let alignment: NSTextAlignment

  public init(
    family: String? = nil,
    name: String? = nil,
    weight: NSFont.Weight = NSFont.Weight.regular,
    size: CGFloat = NSFont.systemFontSize,
    lineHeight: CGFloat? = nil,
    kerning: Double = 0,
    color: NSColor? = nil,
    alignment: NSTextAlignment = .left) {
    self.family = family
    self.name = name
    self.weight = weight
    self.size = size
    self.lineHeight = lineHeight
    self.kerning = kerning
    self.color = color
    self.alignment = alignment
  }

  public func with(
    family: String? = nil,
    name: String? = nil,
    weight: NSFont.Weight? = nil,
    size: CGFloat? = nil,
    lineHeight: CGFloat? = nil,
    kerning: Double? = nil,
    color: NSColor? = nil,
    alignment: NSTextAlignment? = nil
    ) -> TextStyle {
    return TextStyle(
      family: family ?? self.family,
      name: name ?? self.name,
      weight: weight ?? self.weight,
      size: size ?? self.size,
      lineHeight: lineHeight ?? self.lineHeight,
      kerning: kerning ?? self.kerning,
      color: color ?? self.color,
      alignment: alignment ?? self.alignment)
  }

  public lazy var paragraphStyle: NSMutableParagraphStyle = {
    let paragraphStyle = NSMutableParagraphStyle()
    if let lineHeight = lineHeight {
      paragraphStyle.minimumLineHeight = lineHeight
      paragraphStyle.maximumLineHeight = lineHeight
    }
    paragraphStyle.alignment = alignment
    return paragraphStyle
  }()

  public lazy var nsFontDescriptor: NSFontDescriptor = {
    var attributes: [NSFontDescriptor.AttributeName: Any] = [:]
    var family = self.family

    if family == nil && name == nil {
      family = NSFont.systemFont(ofSize: NSFont.systemFontSize).familyName
    }

    if let family = family {
      attributes[NSFontDescriptor.AttributeName.family] = family
    }

    if let name = name {
      attributes[NSFontDescriptor.AttributeName.name] = name
    }

    attributes[NSFontDescriptor.AttributeName.traits] = [
      NSFontDescriptor.TraitKey.weight: weight
    ]

    return NSFontDescriptor(fontAttributes: attributes)
  }()

  public lazy var nsFont: NSFont = {
    return NSFont(descriptor: nsFontDescriptor, size: size) ??
        NSFont.systemFont(ofSize: size, weight: weight)
  }()

  public lazy var attributeDictionary: [NSAttributedStringKey: Any] = {
    var attributes: [NSAttributedStringKey: Any] = [
      .font: nsFont,
      .kern: kerning,
      .paragraphStyle: paragraphStyle
    ]

    if let lineHeight = lineHeight {
      attributes[.baselineOffset] = (lineHeight - nsFont.ascender + nsFont.descender) / 2
    }

    if let color = color {
      attributes[.foregroundColor] = color
    }

    return attributes
  }()

  public func apply(to string: String) -> NSAttributedString {
    return NSAttributedString(
      string: string,
      attributes: attributeDictionary)
  }

  public func apply(to attributedString: NSAttributedString) -> NSAttributedString {
    let styledString = NSMutableAttributedString(attributedString: attributedString)
    styledString.addAttributes(
      attributeDictionary,
      range: NSRange(location: 0, length: styledString.length))
    return styledString
  }

  public func apply(to attributedString: NSMutableAttributedString, at range: NSRange) {
    attributedString.addAttributes(
      attributeDictionary,
      range: range)
  }
}
import Foundation
import UIKit

public class TextStyle {
  public let family: String?
  public let name: String?
  public let weight: UIFont.Weight
  public let size: CGFloat
  public let lineHeight: CGFloat?
  public let kerning: Double
  public let color: UIColor?
  public let alignment: NSTextAlignment

  public init(
    family: String? = nil,
    name: String? = nil,
    weight: UIFont.Weight = UIFont.Weight.regular,
    size: CGFloat = UIFont.systemFontSize,
    lineHeight: CGFloat? = nil,
    kerning: Double = 0,
    color: UIColor? = nil,
    alignment: NSTextAlignment = .left) {
    self.family = family
    self.name = name
    self.weight = weight
    self.size = size
    self.lineHeight = lineHeight
    self.kerning = kerning
    self.color = color
    self.alignment = alignment
  }

  public func with(
    family: String? = nil,
    name: String? = nil,
    weight: UIFont.Weight? = nil,
    size: CGFloat? = nil,
    lineHeight: CGFloat? = nil,
    kerning: Double? = nil,
    color: UIColor? = nil,
    alignment: NSTextAlignment? = nil
    ) -> TextStyle {
    return TextStyle(
      family: family ?? self.family,
      name: name ?? self.name,
      weight: weight ?? self.weight,
      size: size ?? self.size,
      lineHeight: lineHeight ?? self.lineHeight,
      kerning: kerning ?? self.kerning,
      color: color ?? self.color,
      alignment: alignment ?? self.alignment)
  }

  public lazy var paragraphStyle: NSMutableParagraphStyle = {
    let paragraphStyle = NSMutableParagraphStyle()
    if let lineHeight = lineHeight {
      paragraphStyle.minimumLineHeight = lineHeight
      paragraphStyle.maximumLineHeight = lineHeight
    }
    paragraphStyle.alignment = alignment
    return paragraphStyle
  }()

  public lazy var uiFontDescriptor: UIFontDescriptor = {
    var attributes: [UIFontDescriptor.AttributeName: Any] = [:]
    var family = self.family

    if family == nil && name == nil {
      family = UIFont.systemFont(ofSize: UIFont.systemFontSize).familyName
    }

    if let family = family {
      attributes[UIFontDescriptor.AttributeName.family] = family
    }

    if let name = name {
      attributes[UIFontDescriptor.AttributeName.name] = name
    }

    attributes[UIFontDescriptor.AttributeName.traits] = [
      UIFontDescriptor.TraitKey.weight: weight
    ]

    return UIFontDescriptor(fontAttributes: attributes)
  }()

  public lazy var uiFont: UIFont = {
    return UIFont(descriptor: uiFontDescriptor, size: size)
  }()

  public lazy var attributeDictionary: [NSAttributedStringKey: Any] = {
    var attributes: [NSAttributedStringKey: Any] = [
      .font: uiFont,
      .kern: kerning,
      .paragraphStyle: paragraphStyle
    ]

    if let lineHeight = lineHeight {
      attributes[.baselineOffset] = (lineHeight - uiFont.ascender + uiFont.descender) / 4
    }

    if let color = color {
      attributes[.foregroundColor] = color
    }

    return attributes
  }()

  public func apply(to string: String) -> NSAttributedString {
    return NSAttributedString(
      string: string,
      attributes: attributeDictionary)
  }

  public func apply(to attributedString: NSAttributedString) -> NSAttributedString {
    let styledString = NSMutableAttributedString(attributedString: attributedString)
    styledString.addAttributes(
      attributeDictionary,
      range: NSRange(location: 0, length: styledString.length))
    return styledString
  }

  public func apply(to attributedString: NSMutableAttributedString, at range: NSRange) {
    attributedString.addAttributes(
      attributeDictionary,
      range: range)
  }
}